#!/usr/bin/python3

"""Publish manual target/current TCP markers for RViz-only planning."""

import argparse
import math

import rospy
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


def pose_from(values):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[:3]
    rx, ry, rz = values[3:]
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        pose.orientation.w = 1.0
    else:
        scale = math.sin(angle / 2.0) / angle
        pose.orientation.x = rx * scale
        pose.orientation.y = ry * scale
        pose.orientation.z = rz * scale
        pose.orientation.w = math.cos(angle / 2.0)
    return pose


def marker_header(marker, frame):
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.ADD


def target_markers(values, frame):
    pose = pose_from(values)
    origin = Point(pose.position.x, pose.position.y, pose.position.z)
    markers = MarkerArray()

    sphere = Marker()
    marker_header(sphere, frame)
    sphere.ns, sphere.id, sphere.type = "target_tcp", 0, Marker.SPHERE
    sphere.pose = pose
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.060
    sphere.color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
    markers.markers.append(sphere)

    guide = Marker()
    marker_header(guide, frame)
    guide.ns, guide.id, guide.type = "target_tcp", 4, Marker.CYLINDER
    guide.pose.position = Point(origin.x, origin.y, origin.z + 0.125)
    guide.pose.orientation.w = 1.0
    guide.scale.x = guide.scale.y = 0.012
    guide.scale.z = 0.25
    guide.color = ColorRGBA(1.0, 0.0, 1.0, 0.85)
    markers.markers.append(guide)

    label = Marker()
    marker_header(label, frame)
    label.ns, label.id, label.type = "target_tcp", 5, Marker.TEXT_VIEW_FACING
    label.pose.position = Point(origin.x, origin.y, origin.z + 0.275)
    label.pose.orientation.w = 1.0
    label.scale.z = 0.035
    label.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    label.text = "Target TCP"
    markers.markers.append(label)
    return markers


def current_markers(pose, frame):
    markers = MarkerArray()
    sphere = Marker()
    marker_header(sphere, frame)
    sphere.ns, sphere.id, sphere.type = "current_calibrated_tcp", 0, Marker.SPHERE
    sphere.pose = pose
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.040
    sphere.color = ColorRGBA(0.0, 1.0, 1.0, 0.95)
    markers.markers.append(sphere)

    label = Marker()
    marker_header(label, frame)
    label.ns, label.id, label.type = "current_calibrated_tcp", 1, Marker.TEXT_VIEW_FACING
    label.pose.position = Point(
        pose.position.x, pose.position.y, pose.position.z + 0.030,
    )
    label.pose.orientation.w = 1.0
    label.scale.z = 0.030
    label.color = ColorRGBA(0.0, 1.0, 1.0, 1.0)
    label.text = "Current calibrated TCP"
    markers.markers.append(label)
    return markers


def rotate(vector, quaternion):
    """Rotate a vector by a geometry_msgs quaternion."""
    x, y, z = vector
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def multiply_quaternions(first, second):
    ax, ay, az, aw = first.x, first.y, first.z, first.w
    bx, by, bz, bw = second.x, second.y, second.z, second.w
    result = Pose().orientation
    result.x = aw * bx + ax * bw + ay * bz - az * by
    result.y = aw * by - ax * bz + ay * bw + az * bx
    result.z = aw * bz + ax * by - ay * bx + az * bw
    result.w = aw * bw - ax * bx - ay * by - az * bz
    return result


def tcp_pose_from_tool(transform, tcp_offset):
    offset_pose = pose_from(tcp_offset)
    translated = rotate(tcp_offset[:3], transform.transform.rotation)
    pose = Pose()
    pose.position.x = transform.transform.translation.x + translated[0]
    pose.position.y = transform.transform.translation.y + translated[1]
    pose.position.z = transform.transform.translation.z + translated[2]
    pose.orientation = multiply_quaternions(
        transform.transform.rotation,
        offset_pose.orientation,
    )
    return pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", nargs=6, type=float, required=True)
    parser.add_argument("--frame", default="base")
    parser.add_argument("--tool-link", default="tool0")
    parser.add_argument(
        "--tcp-offset",
        nargs=6,
        type=float,
        required=True,
        help="active pendant TCP relative to tool0: x y z rx ry rz (m, rad)",
    )
    args = parser.parse_args()

    rospy.init_node("manual_tcp_marker_publisher")
    target_publisher = rospy.Publisher(
        "/moveit_target_pose_marker", MarkerArray, queue_size=1, latch=True,
    )
    tcp_publisher = rospy.Publisher(
        "/moveit_current_tcp_marker", MarkerArray, queue_size=1, latch=True,
    )
    rospy.sleep(0.2)
    target_publisher.publish(target_markers(args.target, args.frame))
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            tool_transform = tf_buffer.lookup_transform(
                args.frame,
                args.tool_link,
                rospy.Time(0),
                rospy.Duration(0.2),
            )
            tcp_publisher.publish(
                current_markers(
                    tcp_pose_from_tool(tool_transform, args.tcp_offset),
                    args.frame,
                )
            )
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for %s -> %s transform: %s",
                args.frame,
                args.tool_link,
                exc,
            )
        rate.sleep()


if __name__ == "__main__":
    main()
