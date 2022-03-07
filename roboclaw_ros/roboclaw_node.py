#!/usr/bin/env python
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater 

import rclpy
from rclpy.node import Node 
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

from .driver import roboclaw_3 as roboclaw 

import tf_transformations

__author__ = "chrisk+github@vidog.com (Chris Kohlhardt)"
# This code forked from https://github.com/sonyccd/roboclaw_ros
# and adapted to work with Ros2 
#__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):

       
       
        self.node = rclpy.create_node('encoder_odom') 
        self.node.get_logger().debug("create node?")
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width 
        self.odom_pub = self.node.create_publisher(Odometry,self.node.declare_parameter('~topic_odom_out', '/odom' ).value , 10)
        timer_period = 0.5
                 

        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = self.node.get_clock().now()

        self.br = TransformBroadcaster(self.node)


    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = self.node.get_clock().now()
        d_time = (current_time - self.last_enc_time)
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        #if abs(d_time) < 0.000001:
        if abs( d_time.nanoseconds ) < 1000:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / (d_time.nanoseconds / 1000000000 )
            vel_theta = d_theta / ( d_time.nanoseconds / 1000000000 )

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rospy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rospy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf_transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = self.node.get_clock().now().to_msg() 

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'        

        t.transform.translation.x = cur_x
        t.transform.translation.y = cur_y
        t.transform.translation.z = 0.0 

        q = tf_transformations.quaternion_from_euler(0, 0, -cur_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

     # ROS1 code
     #   br.sendTransform((cur_x, cur_y, 0),
     #                    tf_transformations.quaternion_from_euler(0, 0, -cur_theta),
     #                    current_time,
     #                    "base_link",
     #                    "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        # From ros1 code
        #odom.pose.pose.orientation = Quaternion(*quat)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


#class Node:
class RoboClawNode(Node):
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

    
        super().__init__('roboclaw_node')
        #TODO! Missing onshutdown, do I need that?
        self.get_logger().info("Connecting to roboclaw")     

        dev_name = self.declare_parameter('~dev', '/dev/ttyACM0' ).value
        baud_rate = int(self.declare_parameter('~baud', '115200').value) 
        self.address = int(self.declare_parameter('~address', '128' ).value ) 

        self.get_logger().debug("  looking for roboclaw at " + dev_name ) 
        self.get_logger().debug("  baud set to " + str(baud_rate) ) 
        self.get_logger().debug("  address set to " + str(self.address) ) 
        
        if self.address > 135 or self.address < 128:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try: 
            self.rc = roboclaw.Roboclaw( dev_name, baud_rate )
            self.rc.Open()
        except Exception as e: 
            self.get_logger().fatal('Could not connect to Roboclaw')
            self.get_logger().debug(e)
            self.destroy_node()


        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try: 
            version = self.rc.ReadVersion( self.address )
        except Exception as e: 
            self.get_logger().warn("Problem getting roboclaw version") 
            self.get_logger().debug(e) 
            pass

        if not version[0]: 
            self.get_logger().warn( "Could not get version from roboclaw")
        else: 
            self.get_logger().debug(repr(version[1])) 

        self.rc.SpeedM1M2(self.address, 0, 0)
        self.rc.ResetEncoders(self.address) 

        self.MAX_SPEED = float( self.declare_parameter( "~max_speed", "2.0").value )
        self.TICKS_PER_METER = float( self.declare_parameter("~tick_per_meter", "4342.2").value )
        self.BASE_WIDTH = float( self.declare_parameter("~base_width", "0.315").value )

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH) 

        self.last_set_speed_time = self.get_clock().now()

        self.create_subscription( Twist, "/cmd_vel", self.cmd_vel_callback, 10 )

        # TODO Why was there a sleep in here? Do we even need this?    
        #rospy.sleep(1) 

        self.get_logger().debug("dev: " + dev_name)
        self.get_logger().debug("baud: " + str(baud_rate))
        self.get_logger().debug("address: " + str(self.address))
        self.get_logger().debug("max_speed: " + str(self.MAX_SPEED))
        self.get_logger().debug("ticks_per_meter: " +  str(self.TICKS_PER_METER))
        self.get_logger().debug("base_width: " +  str(self.BASE_WIDTH))

        timer_period = 0.1 
        self.timer = self.create_timer( timer_period, self.run )


    def run(self):

        #self.get_logger().info("Starting motor drive")     
        #rospy.loginfo("Starting motor drive")
        #r_time = rospy.Rate(10)
        #while not rospy.is_shutdown():

            self.get_logger().info(" now:" + str(self.get_clock().now()) )
            self.get_logger().info(" last:" + str(self.last_set_speed_time) )
            difer = self.get_clock().now() - self.last_set_speed_time
            self.get_logger().info(" last:" + str(difer) )
            
            if ( self.get_clock().now() - self.last_set_speed_time) > Duration(seconds=1): 
                self.get_logger().info("Did not get command for 1 second, stopping")     
                try:
                    self.rc.ForwardM1(self.address, 0)
                    self.rc.ForwardM2(self.address, 0)
                except OSError as e: 
                    self.get_logger().error("Could not stop")     
                    self.get_logger().debug(e)

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                status1, enc1, crc1 = self.rc.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e: 
                self.get_logger().warn("ReadEncM1 OSError: %d", e.errno)
                self.get_logger().debug(e)

            try:
                status2, enc2, crc2 = self.rc.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e: 
                self.get_logger().warn("ReadEncM2 OSError: %d", e.errno)
                self.get_logger().debug(e)

            if ('enc1' in vars()) and ('enc2' in vars()):
                self.get_logger().debug(" Encoders %d %d" % (enc1, enc2))
                self.encodm.update_publish(enc1, enc2)

                self.updater.update()
            #from Ros1
            #r_time.sleep()

    def cmd_vel_callback(self, twist): 
        self.last_set_speed_time = self.get_clock().now() 

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        self.get_logger().debug("vr_ticks:" + str(vr_ticks) + " vl_ticks: " +  str(vl_ticks))

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks == 0 and vl_ticks == 0:
                self.rc.ForwardM1(self.address, 0)
                self.rc.ForwardM2(self.address, 0)
            else:
                self.rc.SpeedM1M2(self.address, vr_ticks, vl_ticks)
        except OSError as e:
            self.get_logger().warn("SpeedM1M2 OSError: " + e.errno)
            self.get_logger().debug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = self.rc.ReadError(self.address)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            self.get_logger().debug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", str( self.rc.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", str( self.rc.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", str( self.rc.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", str( self.rc.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            self.get_logger().debug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        try:
            self.rc.ForwardM1(self.address, 0)
            self.rc.ForwardM2(self.address, 0)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                self.get_logger().debug(e)



def main(args=None):
    rclpy.init(args=args)
    roboclaw_node = RoboClawNode()
    rclpy.spin(roboclaw_node)
  
   

    roboclaw_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
