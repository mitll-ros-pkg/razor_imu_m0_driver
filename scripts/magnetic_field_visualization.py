#!/usr/bin/python
#

# simple node publishing a Marker for visualizing the magnetic field and calculating its magnitude

import rospy
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class MagneticFieldViz:
    def __init__(self):
        rospy.Subscriber("imu/mag", MagneticField, self.mag_callback)

        self.viz_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.mag_pub = rospy.Publisher("magnetic_field_magnitude", Float64, queue_size=10)

    def mag_callback(self, msg):
        # calculate and publish magnitude
        magnitude = (msg.magnetic_field.x * msg.magnetic_field.x + msg.magnetic_field.y * msg.magnetic_field.y + msg.magnetic_field.z * msg.magnetic_field.z)**(1/2.0)
        self.mag_pub.publish(Float64(magnitude))

        # create visuzalization message
        arrow = Marker()
        arrow.header = msg.header
        arrow.ns = "magnetic field vector (uTesla)"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.2
        arrow.scale.y = 1.0
        arrow.scale.z = 1.0
        arrow.color.r = 0.8
        arrow.color.g = 0.6
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.points = [Point(0, 0, 0), Point(msg.magnetic_field.x*1e6, msg.magnetic_field.y*1e6, msg.magnetic_field.z*1e6)]

        direction = Marker()
        direction.header = msg.header
        direction.ns = "magnetic field unit vector"
        direction.id = 0
        direction.type = Marker.ARROW
        direction.action = Marker.ADD
        direction.scale.x = 0.05
        direction.scale.y = 0.1
        direction.scale.z = 0.1
        direction.color.r = 1.0
        direction.color.g = 0.6
        direction.color.b = 0.6
        direction.color.a = 1.0
        direction.points = [Point(0, 0, 0), Point(msg.magnetic_field.x/magnitude, msg.magnetic_field.y/magnitude, msg.magnetic_field.z/magnitude)]
        
        mag_text = Marker()
        mag_text.header = msg.header
        mag_text.ns = "magnetic field magnitude text"
        mag_text.id = 0
        mag_text.type = Marker.TEXT_VIEW_FACING
        mag_text.action = Marker.ADD
        mag_text.pose.position.x = 0
        mag_text.pose.position.y = 0
        mag_text.pose.position.z = 0
        mag_text.scale.z = 0.1
        mag_text.color.r = 1.0
        mag_text.color.g = 1.0
        mag_text.color.b = 1.0
        mag_text.color.a = 1.0
        mag_text.text = "%.1f uT" % (magnitude*1e6)
        
        # publish the running sum message
        self.viz_pub.publish(MarkerArray([arrow, direction, mag_text]))

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    rospy.init_node("magnetic_field_viz")

    node = MagneticFieldViz()

    # enter the ROS main loop
    rospy.spin()
