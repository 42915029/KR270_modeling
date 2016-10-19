#!/usr/bin/env python

import roslib
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control_publisher')

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()

    pub = rospy.Publisher('/kr270/trajectory_controller/command', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "kr270::socket"

    jt.joint_names.append("joint_a1" )
    jt.joint_names.append("joint_a2" )
    jt.joint_names.append("joint_a3" )
    jt.joint_names.append("joint_a4" )
    jt.joint_names.append("joint_a5" )

    n = 1500
    dt = 0.01
    rps = 0.05
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 =  3*math.sin(theta/50)
        x2 =  math.sin(theta/60) - 0.4
        x3 =  math.sin(theta/70)
        x4 =  3*math.sin(theta/40)
        x5 =  3*math.sin(theta/45)

        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x3)
        p.positions.append(x4)
        p.positions.append(x5)
        jt.points.append(p)

        # set duration
        jt.points[i].time_from_start = rospy.Duration.from_sec(dt * i)
        rospy.loginfo("test: angles[%f][%f, %f]",jt.points[i].time_from_start.to_sec(),x1,x2)

    pub.publish(jt)
    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass
