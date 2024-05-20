#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def tf_echo_publisher():
    rospy.init_node('tf_echo_publisher', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher('/transformed_tf', TransformStamped, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tf_buffer.lookup_transform("base_link", "Gripper_1", rospy.Time(0))
            pub.publish(transform)
            rospy.loginfo("Transform published successfully")
        except tf2_ros.LookupException as e:
            rospy.logwarn("Failed to lookup transform: %s", str(e))
        except tf2_ros.ConnectivityException as e:
            rospy.logwarn("Connectivity error: %s", str(e))
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("Extrapolation error: %s", str(e))
        except rospy.ROSException as e:
            rospy.logerr("ROS exception: %s", str(e))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_echo_publisher()
    except rospy.ROSInterruptException:
        pass

