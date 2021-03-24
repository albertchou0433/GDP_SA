#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

t = Odometry()

def callback(msg):

    k = msg.pose[2]

    t.header.stamp = rospy.Time.now()
    t.pose.pose.position.x = k.position.x
    t.pose.pose.position.y = k.position.y
    t.pose.pose.position.z = k.position.z


def main():

    rospy.init_node("topic_publisher")

    sub=rospy.Subscriber("/gazebo/model_states",ModelStates,callback)

    pub=rospy.Publisher("time_xyz",Odometry,queue_size=10)

    pub.publish(t)

    rospy.spin()

if __name__== "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
