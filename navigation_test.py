#!/usr/bin/env python

import rospy
import subprocess
import signal

# Launch a new simulation

s_params = {
    "cur_speed": [0.2, 0.4, 0.7, 1.2, 1.4],
    "ra": [10, 20, 30, 40, 50],
    "tug_speed": [1.0, 3.0, 4.0, 5.0, 7.0],
    "obs_speed": [0, 2.0, 3.0, 4.0, 6.0]
}

"""
Function to launch a new simulation with the specified parameters.

:param params: Parameters to change before starting the simulation. The parameters should be
a dictionary with the shape:

params = {
    "cur_speed": [],
    "ra": [],
    "tug_speed": [],
    "obs_speed": []
}

The arrays should have the same number of elements.
"""
def launch_simulation(params):
    child = subprocess.Popen(["roslaunch","collision_avoidance_in_singapore","two_obstacles_head.launch"])
    #child.wait() #You can use this line to block the parent process untill the child process finished.
    # Printing process information
    print("parent process")
    print(child.poll())

    rospy.loginfo('The PID of child: %d', child.pid)
    print ("The PID of child:", child.pid)

rospy.sleep(30)

child.send_signal(signal.SIGINT) #You may also use .terminate() method
#child.terminate()

#for more: https://docs.python.org/2/library/subprocess.html


if __name__== "__main__":

    for cur_speed_value in s_params["cur_speed"]:
        rospy.set_param("/asv/asv/Vx_current", cur_speed_value)
        rospy.set_param("/obstacles/ship1/ship1/Vx_current", cur_speed_value)
        rospy.set_param("/obstacles/ship1/ship1/Vx_current", cur_speed_value)
        for cur_speed_value in s_params["ra"]:
            for tug_speed_value in s_params["tug_speed"]:
                rospy.set_param("/asv/LOSNode/u_d", tug_speed_value)
                for obs_speed_value in s_params["obs_speed"]:
                    pass
                    #launch_simulation(params)