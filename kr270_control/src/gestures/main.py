#!/usr/bin/env python

import os, sys
import signal
import roslib
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def sigin_handler(signum, frame):
        print "\n"
        sys.exit()

signal.signal(signal.SIGINT, sigin_handler)

def jointTrajectoryCommand():
    # Initialize the node

    print "Initialising ROS."

    try:
        rospy.init_node('joint_control_publisher')
        pub = rospy.Publisher('/kr270/trajectory_controller/command', JointTrajectory, queue_size=10)

    except Exception, e:
        print "ROS could not be initialised."
        raise
    else:
        print "ROS successfully initialised."
    
    avail_traj_str = "{"

    traj_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "generators/trajectories/")

    avail_traj = []

    for file in os.listdir(traj_dir_path):
        if file.endswith("_trajectory.traj"):
            avail_traj.append(file[:-16])
            avail_traj_str = avail_traj_str + file[:-16] + ", "

    avail_traj_str = avail_traj_str[:-2] + "}"
  
    while True:
        selected_traj = ""
        
        while selected_traj == "":

            print "Available gestures: " + avail_traj_str

            traj_name = raw_input('Please select gesture to be performed: ')

            if traj_name == "quit" or traj_name == "q" or traj_name == "exit":
                sys.exit()

            for traj in avail_traj:
                if traj_name == traj:
                    selected_traj = traj_name

            if selected_traj == "":
                print "Unknown gesture.\n"

        print "Performing gesture: " + selected_traj

        jt = JointTrajectory()

        jt.header.stamp = rospy.Time.now()
        jt.header.frame_id = "kr270::socket"

        jt.joint_names.append("joint_a1")
        jt.joint_names.append("joint_a2")
        jt.joint_names.append("joint_a3")
        jt.joint_names.append("joint_a4")
        jt.joint_names.append("joint_a5")

        for file in os.listdir(traj_dir_path):
            if file == (selected_traj + "_trajectory.traj"):
                f = open(os.path.join(traj_dir_path, file), "r")
                break

        if f is None:
            print ".traj file could not be opened."
            continue

        n = int(f.readline())
        duration = 0

        for line in f:
            p = JointTrajectoryPoint()

            x1, x2, x3, x4, x5, t = [float(x) for x in line.split(",")]

            p.positions.append(x1)
            p.positions.append(x2)
            p.positions.append(x3)
            p.positions.append(x4)
            p.positions.append(x5)
            p.time_from_start = rospy.Duration.from_sec(3+t)
            duration = 3 + t
            jt.points.append(p)

        pub.publish(jt)

        # wait for gesture to finish
        print "Performing gesture..."
        time.sleep(duration + 0.5)


    rospy.spin()

if __name__ == '__main__':
    try:
        jointTrajectoryCommand()
    except rospy.ROSInterruptException: pass


