#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("fake_pose_publisher", anonymous=False)

    pose_topic = rospy.get_param("~pose_topic", "/vrpn_client_node/tello/pose")
    frame_id = rospy.get_param("~frame_id", "world")
    rate_hz = float(rospy.get_param("~rate_hz", 20.0))

    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 1.0))

    qx = float(rospy.get_param("~qx", 0.0))
    qy = float(rospy.get_param("~qy", 0.0))
    qz = float(rospy.get_param("~qz", 0.0))
    qw = float(rospy.get_param("~qw", 1.0))

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo(
        "fake_pose_publisher started topic=%s rate=%.1f xyz=(%.2f, %.2f, %.2f)",
        pose_topic,
        rate_hz,
        x,
        y,
        z,
    )

    while not rospy.is_shutdown():
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()