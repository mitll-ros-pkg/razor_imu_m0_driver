#!/usr/bin/python
#

# simple node publishing transform corresponding to the IMU orientation

import rospy
from sensor_msgs.msg import Imu
import tf

class IMUToTF:
    def __init__(self):
        self.world_frame_id = rospy.get_param('~world_frame_id', 'world')
        self.debug = rospy.get_param('~debug', False)

        rospy.Subscriber("imu/data", Imu, self.imu_callback)

        self.tfb = tf.TransformBroadcaster()

        # self.yaw is used for integrating z-axis angular rate for debugging
        self.yaw = 0

    def imu_callback(self, msg):
        if self.debug:
            # print quaternion values to screen
            mag = (msg.orientation.x**2.0 + msg.orientation.y**2.0 + msg.orientation.z**2.0 + msg.orientation.w**2.0)**(1/2.0)
            print "qx: %.2f qy: %.2f qz: %.2f qw: %.2f magnitude: %.2f" % (mag, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            # print euler angles to screen
            euler = tf.transformations.euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
            print "roll: %.2f pitch: %.2f yaw: %.2f" % (euler[0]*57.3, euler[1]*57.3, euler[2]*57.3)
            # integrate z axis angular rate and print
            self.yaw = self.yaw + msg.angular_velocity.z/50.0
            print "yaw (integrated z axis): %.2f" % (self.yaw*57.3)

        self.tfb.sendTransform((0, 0, 0),
                               (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                               rospy.Time.now(),
                               msg.header.frame_id,
                               self.world_frame_id)

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("imu_orientation_to_tf")

    node = IMUToTF()

    # enter the ROS main loop
    rospy.spin()
