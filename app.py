#!/usr/bin/env python3

import ntcore
import rospy


class ROSSubscriberNode:
    # https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart
    # this might be the wrong topic name, could be "tag_detections"
    ROS_TOPIC_NAME = "tf"
    DATA_TYPE = "tf2_msgs/TFMessage"
    
    @staticmethod
    def DEFAULT_CALLBACK(data):
        rospy.loginfo(f"I heard {data.data}")

    def __init__(self, callback_function=None, networktables_topic_name=None, data_type=None):
        callback_function = callback_function or self.DEFAULT_CALLBACK
        ros_topic_name    = ros_topic_name    or self.ROS_TOPIC_NAME
        data_type         = data_type         or self.DATA_TYPE

        rospy.init_node(f"Listener@{callback.__name__}")
        rospy.Subscriber(ros_topic_name, data_type, callback)
    
    def spin(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


class NetworkTablesSender:
    # The topic name we publish to the networktables
    # this is arbitrary and chosen by us.
    NETWORKTABLES_TOPIC_NAME = "translation_rotation"
    # the table name the topic is in.
    # this is also arbitrary, and could need to be changed if other scripts expect it to be 
    # named something different
    TABLE_NAME = "datatable"


    def __init__(self, *, ros_topic_name=None, table_name=None):
        # Almost certainly we won't need to pass args to change the defaults
        # but it's there just in case :/
        networktables_topic_name = networktables_topic_name or self.NETWORKTABLES_TOPIC_NAME
        table_name               = table_name               or self.TABLE_NAME

        # Get the default instance of NetworkTables that was created automatically
        # when the robot program starts

        # Get the table within that instance that contains the data. There can
        # be as many tables as you like and exist to make it easier to organize
        # your data. In this case, it's a table called datatable.

        # Start publishing topics within that table that correspond to the X and Y values
        # for some operation in your program.
        # The topic names are actually "/datatable/x" and "/datatable/y".
        self.double_array_publisher = \
            ntcore.NetworkTableInstance\
                .getDefault()\
                .getTable(ros_topic_name)\
                .getDoubleArrayTopic(networktables_topic_name)\
                .publish()

    @staticmethod
    def tf_message_to_double_array(tf_message):
        # This is the only function that really 'does' anything.
        # Convert the tf_message object to an array of doubles
        # so that it can be sent to the Java code.
        return [
            tf_message.transform.translation.x,
            tf_message.transform.translation.y,
            tf_message.transform.translation.z,
            tf_message.transform.rotation.x,
            tf_message.transform.rotation.y,
            tf_message.transform.rotation.z,
            tf_message.transform.rotation.w,
        ]

    def __call__(self, tf_message):
        # Convert tf_message to a a double array and publish it to Networktables
        # Central function along with `tf_message_to_double_array`
        self.double_array_publisher.set(
            self.tf_message_to_double_array(
                tf_message
            )
        )

def main():
    ROSSubscriberNode( NetworkTablesSender() ).spin()

if __name__ == "__main__":
    main()