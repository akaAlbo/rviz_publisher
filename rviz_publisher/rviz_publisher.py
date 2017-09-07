#!/usr/bin/python

"""
Created on Sep 6, 2017

@author: flg-ma
@attention: Auto Position Publisher for RVIZ
@contact: marcel.albus@ipa.fraunhofer.de (Marcel Albus)
@version: 1.1.0
"""

import rospy
import argparse
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import tf


# colors in terminal prints
class TerminalColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



class RvizPublisher():
    def __init__(self):
        self.tc = TerminalColors()
        # topics you can publish
        self.topics = {'/initialpose': PoseWithCovarianceStamped,
                       '/move_base_simple/goal': PoseStamped,
                       '/clicked_point': PointStamped}
        self.publisher = {}
        for topic, msg_type in self.topics.iteritems():
            self.publisher[topic] = rospy.Publisher(topic, msg_type, queue_size=2)

        # setup argument parser
        self.args = self.build_parser().parse_args()

        rospy.init_node('rviz_publisher', anonymous=True)
        # python bug... sleep NEEDED!(ros tired...) Min: 0.5 sec
        rospy.sleep(.5)

    def build_parser(self):
        parser = argparse.ArgumentParser(
            description='Publish \'/initialpose\' for RVIZ to auto-locate robot')
        # group = parser.add_mutually_exclusive_group()
        parser.add_argument('-g', '--goal', help='goal for robot as: x y R P Y', nargs='+', type=float)
        parser.add_argument('launch', help='launch-file to read \'initial_config\' from', type=str)
        return parser

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
        # get roll, pitch, yaw from additional parameters
        roll = args[0]
        pitch = args[1]
        yaw = args[2]
        # convert euler-angles to quaternions
        quaternion = self.euler2quaternion(roll, pitch, yaw)

        if msg_type is PoseWithCovarianceStamped:
            msg.pose.pose.position.x = position_x
            msg.pose.pose.position.y = position_y
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
        # generate message to publish
        msg = self.setupMessage(self.topics[topic], 'map', pose_x, pose_y, *args)
        print msg
        self.publisher[topic].publish(msg)

    def getParams(self):
        filename = self.args.launch
        with open(filename, 'r') as f:
            content = f.readlines()

        position = {}
        for line in content:
            if 'initial_config' in line and 'default' in line:
                # print line
                position['x'] = line[line.index('-x') + 3:line.index('-y') - 1]
                position['y'] = line[line.index('-y') + 3:line.index('-R') - 1]
                position['R'] = line[line.index('-R') + 3:line.index('-P') - 1]
                position['P'] = line[line.index('-P') + 3:line.index('-Y') - 1]
                position['Y'] = line[line.index('-Y') + 3:line.index('"/>')]
                # convert str to float
                for pos in position:
                    position[pos] = float(position[pos])
        return position

    def main(self):
        # publish initialpose as [x, y, roll, pitch, yaw]
        output = 'RVIZ auto-position publisher with goal: [x: ' + \
                 str(self.args.goal[0]) + '; y: ' + \
                 str(self.args.goal[1]) + '; R: ' + \
                 str(self.args.goal[2]) + '; P: ' + \
                 str(self.args.goal[3]) + '; Y: ' + \
                 str(self.args.goal[4]) + ']'
        print self.tc.OKBLUE + '=' * output.__len__()
        print output
        print '=' * output.__len__() + self.tc.ENDC
        position = rp.getParams()
        rp.publish('/initialpose', position['x'], position['y'], position['R'], position['P'], position['Y'])
        print self.tc.OKBLUE + '='*80 + self.tc.ENDC


        rospy.sleep(3)
        rp.publish('/move_base_simple/goal', self.args.goal[0], self.args.goal[1], self.args.goal[2], self.args.goal[3],
                   self.args.goal[4])
        print self.tc.OKBLUE + '='*80 + self.tc.ENDC


if __name__ == '__main__':
    try:
        rp = RvizPublisher()
        rp.main()
    except rospy.ROSInterruptException:
        pass
