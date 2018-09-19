#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from dynamic_reconfigure.server import Server
import rospy
from geometry_msgs.msg import PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_rviz_plugins.cfg import ClassificationResultVisualizerConfig
import message_filters as MF
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf

class ClassificationResultVisualizer(ConnectionBasedTransport):
    def __init__(self):
        super(ClassificationResultVisualizer, self).__init__()
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.srv = Server(ClassificationResultVisualizerConfig,
                          self.config_callback)
        self.pub_marker = self.advertise("~output", MarkerArray, queue_size=10)
        self.listener = tf.TransformListener()

    def subscribe(self):
        sub_cls = MF.Subscriber(
            "~input/colors", ClassificationResult, queue_size=10)
        sub_box = MF.Subscriber(
            "~input/boxes", BoundingBoxArray, queue_size=10)
        sub_od = MF.Subscriber(
            "~input/objects", ClassificationResult, queue_size=10)
        sync_box = MF.ApproximateTimeSynchronizer([sub_box,sub_cls,sub_od], self.queue_size,slop=0.1)
        sync_box.registerCallback(self.box_msg_callback)
        self.syncs = [sync_box]
        self.subscribers = [sub_cls, sub_box,sub_od]

    def unsubscribe(self):
        for sub in self.subscribers:
            sub.unregister()
        self.syncs = []

    def config_callback(self, config, level):
        self.text_color = {'r': config.text_color_red,
                           'g': config.text_color_green,
                           'b': config.text_color_blue,
                           'a': config.text_color_alpha}
        self.text_offset = [config.text_offset_x,
                            config.text_offset_y,
                            config.text_offset_z]
        self.text_size = config.text_size
        self.marker_lifetime = config.marker_lifetime
        return config

    def box_msg_callback(self, bboxes, colors,objects):
        msg = MarkerArray()
        for i, data in enumerate(zip(bboxes.boxes, zip(colors.label_names, colors.label_proba),zip(objects.label_names, objects.label_proba))):
            bbox, zip_colors, zip_objects = data

            text_objects = "%s (%.3f)" % zip_objects
            text_colors = "%s (%.3f)" % zip_colors
	    #world_pose = self.listener.transformPose('yumi_base_link','camera_color_optical_frame', bbox.pose)
            text_colors = text_objects + '\n' + text_colors #+ '\n' + world_pose.position.x + ' , ' + world_pose.position.y
            j=i

            m_colors = Marker(type=Marker.TEXT_VIEW_FACING,
                       action=Marker.MODIFY,
                       header=bbox.header,
                       id=j,
                       pose=bbox.pose,
                       color=ColorRGBA(**self.text_color),
                       text=text_colors,
                       ns=colors.classifier,
                       lifetime=rospy.Duration(self.marker_lifetime))
            m_colors.scale.z = self.text_size
            m_colors.pose.position.x += self.text_offset[0]
            m_colors.pose.position.y += self.text_offset[1]
            m_colors.pose.position.z += self.text_offset[2]
            msg.markers.append(m_colors)

            '''m_objects = Marker(type=Marker.TEXT_VIEW_FACING,
                       action=Marker.MODIFY,
                       header=bbox.header,
                       id=j+1,
                       pose=bbox.pose,
                       color=ColorRGBA(**self.text_color),
                       text=text_objects,
                       ns=objects.classifier,
                       lifetime=rospy.Duration(self.marker_lifetime))
            m_objects.scale.z = self.text_size
            m_objects.pose.position.x += self.text_offset[0]
            m_objects.pose.position.y += (self.text_offset[1]-0.1)
            m_objects.pose.position.z += self.text_offset[2]
            msg.markers.append(m_objects)'''

        self.pub_marker.publish(msg)


if __name__ == '__main__':
    rospy.init_node("classification_result_visualizer")
    viz = ClassificationResultVisualizer()
    rospy.spin()
