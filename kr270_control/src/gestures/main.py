#!/usr/bin/env python

import roslib
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control_publisher')



    pub = rospy.Publisher('/kr270/trajectory_controller/command', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "kr270::socket"

    jt.joint_names.append("joint_a1" )
    jt.joint_names.append("joint_a2" )
    jt.joint_names.append("joint_a3" )
    jt.joint_names.append("joint_a4" )
    jt.joint_names.append("joint_a5" )

    n = 201
    dt = 0.05
    for i in range (n):
        if i <= 50:
            p = JointTrajectoryPoint()
            x1 =  0.05*i
            x2 =  0.05*i
            x3 =  0.05*i
            x4 =  0.1*i
            x5 =  0.05*i

            p.positions.append(x1)
            p.positions.append(x2)
            p.positions.append(x3)
            p.positions.append(x4)
            p.positions.append(x5)
            jt.points.append(p)

            # set duration
            jt.points[i].time_from_start = rospy.Duration.from_sec(dt * (i+1))
        else:
            if i <= 150:
                p = JointTrajectoryPoint()
                x1 =  0.05*(100-i)
                x2 =  0.05*(100-i)
                x3 =  0.05*(100-i)
                x4 =  0.1*(100-i)
                x5 =  0.05*(100-i)

                p.positions.append(x1)
                p.positions.append(x2)
                p.positions.append(x3)
                p.positions.append(x4)
                p.positions.append(x5)
                jt.points.append(p)

                # set duration
                jt.points[i].time_from_start = rospy.Duration.from_sec(dt * (i+1))
            else:
                p = JointTrajectoryPoint()
                x1 =  -0.05*(200-i)
                x2 =  -0.05*(200-i)
                x3 =  -0.05*(200-i)
                x4 =  -0.1*(200-i)
                x5 =  -0.05*(200-i)

                p.positions.append(x1)
                p.positions.append(x2)
                p.positions.append(x3)
                p.positions.append(x4)
                p.positions.append(x5)
                jt.points.append(p)

                # set duration
                jt.points[i].time_from_start = rospy.Duration.from_sec(dt * (i+1))

    print "start waiting"
    time.sleep(3)
    print"stop waiting"

    pub.publish(jt)
    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass


