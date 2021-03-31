#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class DumpOdom:

  def __init__(self):

    # Node subscribers
    self.sub = rospy.Subscriber('odom', Odometry, self.odomCb)

  def odomCb(self, msg):
    quat = msg.pose.pose.orientation
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    print(str(rospy.get_time()) + '\t' + 
        #str(rospy.Time.now()) + '\t' +
        str(msg.pose.pose.position.x) + '\t' + 
        str(msg.pose.pose.position.y) + '\t' + str(yaw) + '\t' +
        str(msg.twist.twist.linear.x) + '\t' +
        str(msg.twist.twist.angular.z))

if __name__ == '__main__':
  rospy.init_node('dump_odom')
  do = DumpOdom()
  rospy.spin()

