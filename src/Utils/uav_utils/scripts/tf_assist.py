#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from math import pi
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped


class OdometryConverter:

    def __init__(self, node, frame_id_in_, frame_id_out_, broadcast_tf_, body_frame_id_, intermediate_frame_id_, world_frame_id_):
        self.node = node
        self.frame_id_in = frame_id_in_
        self.frame_id_out = frame_id_out_
        self.broadcast_tf = broadcast_tf_
        self.body_frame_id = body_frame_id_
        self.intermediate_frame_id = intermediate_frame_id_
        self.world_frame_id = world_frame_id_
        self.in_odom_sub = None
        self.out_odom_pub = None
        self.out_path_pub = None
        self.path_pub_timer = None
        self.tf_pub_flag = True
        if self.broadcast_tf:
            self.node.get_logger().info('ROSTopic: [%s]->[%s] TF: [%s]-[%s]-[%s]' %
                          (self.frame_id_in, self.frame_id_out, self.body_frame_id, self.intermediate_frame_id, self.world_frame_id))
        else:
            self.node.get_logger().info('ROSTopic: [%s]->[%s] No TF' %
                          (self.frame_id_in, self.frame_id_out))

        self.path = []

    def _make_transform(self, stamp, child_frame, parent_frame, translation, quaternion):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        return t

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = euler_from_quaternion(q, 'rzyx')
        wqb = quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        assert(in_odom_msg.header.frame_id == self.frame_id_in)
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            identity_q = quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx')

            if not self.frame_id_in == self.frame_id_out:
                t = self._make_transform(odom_msg.header.stamp,
                    self.frame_id_in, self.frame_id_out,
                    (0.0, 0.0, 0.0), identity_q)
                self.node.br.sendTransform(t)

            if not self.world_frame_id == self.frame_id_out:
                t = self._make_transform(odom_msg.header.stamp,
                    self.world_frame_id, self.frame_id_out,
                    (0.0, 0.0, 0.0), identity_q)
                self.node.br.sendTransform(t)

            t = self._make_transform(odom_msg.header.stamp,
                self.body_frame_id, self.world_frame_id,
                (p[0], p[1], p[2]), wqb)
            self.node.br.sendTransform(t)

            t = self._make_transform(odom_msg.header.stamp,
                self.intermediate_frame_id, self.world_frame_id,
                (p[0], p[1], p[2]), wqc)
            self.node.br.sendTransform(t)

        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = float(p[0])
        pose.pose.position.y = float(p[1])
        pose.pose.position.z = float(p[2])
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        self.path.append(pose)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True


class TfAssistNode(Node):
    def __init__(self):
        super().__init__('tf_assist')

        self.br = TransformBroadcaster(self)
        self.converters = []
        index = 0

        while True:
            prefix = "converter%d." % index
            try:
                frame_id_in = self.declare_parameter(prefix + 'frame_id_in', '').get_parameter_value().string_value
                if not frame_id_in:
                    raise KeyError(prefix + 'frame_id_in')
                frame_id_out = self.declare_parameter(prefix + 'frame_id_out', '').get_parameter_value().string_value
                if not frame_id_out:
                    raise KeyError(prefix + 'frame_id_out')
                broadcast_tf = self.declare_parameter(prefix + 'broadcast_tf', False).get_parameter_value().bool_value
                body_frame_id = self.declare_parameter(prefix + 'body_frame_id', 'body').get_parameter_value().string_value
                intermediate_frame_id = self.declare_parameter(prefix + 'intermediate_frame_id', 'intermediate').get_parameter_value().string_value
                world_frame_id = self.declare_parameter(prefix + 'world_frame_id', 'world').get_parameter_value().string_value

                converter = OdometryConverter(
                    self, frame_id_in, frame_id_out, broadcast_tf, body_frame_id, intermediate_frame_id, world_frame_id)

                topic_prefix = 'converter%d/' % index
                converter.in_odom_sub = self.create_subscription(
                    Odometry, topic_prefix + 'in_odom', converter.in_odom_callback, 10)
                converter.out_odom_pub = self.create_publisher(
                    Odometry, topic_prefix + 'out_odom', 10)
                converter.out_path_pub = self.create_publisher(
                    Path, topic_prefix + 'out_path', 10)

                converter.tf_pub_timer = self.create_timer(
                    0.1, converter.tf_pub_callback)

                converter.path_pub_timer = self.create_timer(
                    0.5, converter.path_pub_callback)

                self.converters.append(converter)
                index += 1
            except KeyError as e:
                if index == 0:
                    raise KeyError(e)
                else:
                    if index == 1:
                        self.get_logger().info(
                            'prefix:"%s" not found. Generate %d converter.' % (prefix, index))
                    else:
                        self.get_logger().info(
                            'prefix:"%s" not found. Generate %d converters' % (prefix, index))
                    break


def main(args=None):
    rclpy.init(args=args)
    node = TfAssistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
