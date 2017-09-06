#!/usr/bin/python

"""
Created on Sep 6, 2017

@author: flg-ma
@attention: Auto Position Publisher for RVIZ
@contact: marcel.albus@ipa.fraunhofer.de (Marcel Albus)
@version: 1.0.0
"""

import rospy
from geneus.generate import msg_type
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import tf


class RvizPublisher():
    def __init__(self):
        self.topics = {'/initialpose': PoseWithCovarianceStamped,
                       '/move_base_simple/goal': PoseStamped,
                       '/clicked_point': PointStamped}
        self.publisher = {}
        for topic, msg_type in self.topics.iteritems():
            self.publisher[topic] = rospy.Publisher(topic, msg_type, queue_size=2)

        # self.publisher_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        # self.publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2)
        # self.publisher_clicked_point = rospy.Publisher('/clicked_point', PointStamped, queue_size=2)

        rospy.init_node('rviz_publisher', anonymous=True)
        # python bug... sleep NEEDED! Min: 0.5 sec
        rospy.sleep(.5)

    def euler2quaternion(self, roll, pitch, yaw):
        # converts euler to quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return quaternion

    def setupMessage(self, msg_type, frame_id, position_x, position_y, *args):
        # frame_id default mostly 'map'
        msg = msg_type()
        msg.header.seq = 1
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        args = list(args)
        roll = args[0]
        pitch = args[1]
        yaw = args[2]
        quaternion = self.euler2quaternion(roll, pitch, yaw)

        if msg_type is PoseWithCovarianceStamped:
            msg.pose.pose.position.x = position_x
            msg.pose.pose.position.y = position_y
            print msg.pose.pose.position.y
            msg.pose.pose.orientation.x = quaternion[0]
            msg.pose.pose.orientation.y = quaternion[1]
            msg.pose.pose.orientation.z = quaternion[2]
            msg.pose.pose.orientation.w = quaternion[3]
            # needed to prevent robot from drifting off
            msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.06853891945200942]
        elif msg_type is PoseStamped:
            msg.pose.position.x = position_x
            msg.pose.position.y = position_y
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]

        elif msg_type is PointStamped:
            msg.point.x = position_x
            msg.point.y = position_y
        return msg

    def publish(self, topic, pose_x, pose_y, *args):
        msg = self.setupMessage(self.topics[topic], 'map', pose_x, pose_y, *args)
        print msg
        # if topic is '/initialpose':
        #     self.publisher_initialpose.publish(msg)
        # elif topic is '/move_base_simple/goal':
        #     self.publisher_goal.publish(msg)
        # elif topic is '/clicked_point':
        #     self.publisher_clicked_point.publish(msg)

        self.publisher[topic].publish(msg)


if __name__ == '__main__':
    try:
        rp = RvizPublisher()
        # rp.publish('/initialpose', .7, 0, 0, 0, 0)
        # rospy.sleep(2)
        rp.publish('/move_base_simple/goal', 4, 0, 0, 0, 0)
        # rp.publish(PointStamped, 0, 0, 0, 0, 0)
    except rospy.ROSInterruptException:
        pass
