#!/usr/bin/env python3
"""Live ROS regression test for the LDOP dynamic-marker output wiring."""

import sys

import rospy
from visualization_msgs.msg import MarkerArray


def main() -> int:
    rospy.init_node("test_ldop_output_publish", anonymous=True)
    try:
        message = rospy.wait_for_message(
            "/ldop/dynamic_object_markers", MarkerArray, timeout=5.0
        )
    except rospy.ROSException as error:
        print(f"FAIL: no dynamic marker message received: {error}")
        return 1

    if not message.markers:
        print("FAIL: dynamic marker message contains no markers")
        return 1

    print(f"PASS: received {len(message.markers)} dynamic markers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
