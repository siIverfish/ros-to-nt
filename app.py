#!/usr/bin/env python3

import ntcore
import rospy

# https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart
# this might be the wrong topic name, could be "tag_detections"
TOPIC_NAME = "tf"
DATA_TYPE = "tf2_msgs/TFMessage"
DEFAULT_CALLBACK = lambda data: rospy.loginfo(f"I heard {data.data}")

def listener(callback=DEFAULT_CALLBACK):
    rospy.init_node(f"Listener@{callback.__name__}")
    rospy.Subscriber(TOPIC_NAME, DATA_TYPE, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

class NetworkTablesSender:
    def __init__(self):
        # Get the default instance of NetworkTables that was created automatically
        # when the robot program starts

        # Get the table within that instance that contains the data. There can
        # be as many tables as you like and exist to make it easier to organize
        # your data. In this case, it's a table called datatable.

        # Start publishing topics within that table that correspond to the X and Y values
        # for some operation in your program.
        # The topic names are actually "/datatable/x" and "/datatable/y".
        self.translation_rotation_publisher = \
            ntcore.NetworkTableInstance\
                .getDefault()\
                .getTable("datatable")\
                .getDoubleArrayTopic("translation_rotation")\
                .publish()


    def __call__(self, tf_message):
        translation_rotation = [
            tf_message.transform.translation.x,
            tf_message.transform.translation.y,
            tf_message.transform.translation.z,
            tf_message.transform.rotation.x,
            tf_message.transform.rotation.y,
            tf_message.transform.rotation.z,
            tf_message.transform.rotation.w,
        ]

        self.translation_rotation_publisher.set(translation_rotation)



if __name__ == "__main__":
    listener(NetworkTablesSender())