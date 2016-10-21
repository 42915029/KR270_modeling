#!/usr/bin/env python

import os
import roslib
import rospy, math, time

from trajectory_msgs.msg import JointTrajectoryPoint

def generateTrajectory():

    #----------------------- Define trajectory ----------------------------#
    jt = []

    n = 201
    dt = 0.05
    for i in range (n):
        if i <= 50:
            p = JointTrajectoryPoint()
            x1 =  0.05*i
            x2 =  0.05*i
            x3 =  0.05*i
            x4 =  0
            x5 =  0

            p.positions.append(x1)
            p.positions.append(x2)
            p.positions.append(x3)
            p.positions.append(x4)
            p.positions.append(x5)
            jt.append(p)

            # set duration
            jt[i].time_from_start = rospy.Duration.from_sec(dt * (i+1))
        else:
            if i <= 150:
                p = JointTrajectoryPoint()
                x1 =  0.05*(100-i)
                x2 =  0.05*(100-i)
                x3 =  0.05*(100-i)
                x4 =  0
                x5 =  0

                p.positions.append(x1)
                p.positions.append(x2)
                p.positions.append(x3)
                p.positions.append(x4)
                p.positions.append(x5)
                jt.append(p)

                # set duration
                jt[i].time_from_start = rospy.Duration.from_sec(dt * (i+1))
            else:
                p = JointTrajectoryPoint()
                x1 =  -0.05*(200-i)
                x2 =  -0.05*(200-i)
                x3 =  -0.05*(200-i)
                x4 =  -0
                x5 =  -0

                p.positions.append(x1)
                p.positions.append(x2)
                p.positions.append(x3)
                p.positions.append(x4)
                p.positions.append(x5)
                jt.append(p)

                # set duration
                jt[i].time_from_start = rospy.Duration.from_sec(dt * (i+1))

    saveTrajectory(jt)

def saveTrajectory(jt):
    rel_path = 'trajectories/' + traj_name + '_trajectory.traj'
    traj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), rel_path)

    f = open(traj_path, 'w+')

    f.write(str(len(jt)) + '\n')

    for point in jt:
        f.write(str(point.positions[0]) + ',' +
                str(point.positions[1]) + ',' +
                str(point.positions[2]) + ',' +
                str(point.positions[3]) + ',' +
                str(point.positions[4]) + ',' +
                str(point.time_from_start.to_sec()) + '\n')

    f.close()


if __name__ == '__main__':

    traj_name = __file__[11:-13]

    try:
        generateTrajectory()
    except:
        print "Generation failed."
        raise
    else:
        print "Trajectory generation successful (" + traj_name + ')'


