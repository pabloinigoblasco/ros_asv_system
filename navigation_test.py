#!/usr/bin/env python

import csv
import os.path
import rospy
import subprocess
import signal
import scipy
import sys

# Import the datatype for the messages
from geometry_msgs.msg import PoseStamped

# Import the euclidean distance function
from scipy.spatial import distance

# Launch a new simulation

s_params = {
    "cur_speed": [0.2, 0.4, 0.7, 1.2, 1.4],
    "ra": [10, 20, 30, 40, 50],
    "tug_speed": [1.0, 3.0, 4.0, 5.0, 7.0],
    "obs_speed": [0, 2.0, 3.0, 4.0, 6.0]
}

"""
Function to launch a new simulation.

:returns: A handler of the launched child process.
"""
def launch_simulation():
    child = subprocess.Popen(["roslaunch","collision_avoidance_in_singapore","two_obstacles_head.launch"])
    #child.wait() #You can use this line to block the parent process untill the child process finished.
    # Printing process information
    print("parent process")
    print(child.poll())

    rospy.loginfo('The PID of child: %d', child.pid)
    print ("The PID of child:", child.pid)

    return child

"""
Helper function to set current parameters to be used during one iteration.
"""
def set_cur_params(cur_speed, cur_ra, cur_tug_speed, cur_obs_speed):
    # Setting current speed
    rospy.set_param("/asv/asv/Vx_current", cur_speed)
    rospy.set_param("/obstacles/ship1/ship1/Vx_current", cur_speed)
    rospy.set_param("/obstacles/ship1/ship1/Vx_current", cur_speed)

    # Setting acceptance radius
    rospy.set_param("/asv/asv/Vx_current", cur_ra)
    rospy.set_param("/obstacles/ship1/LOSNode/acceptance_radius", cur_ra)
    rospy.set_param("/obstacles/ship2/LOSNode/acceptance_radius", cur_ra)

    # Setting tug speed
    rospy.set_param("/asv/LOSNode/u_d", cur_tug_speed)

    # Setting obs speed
    rospy.set_param("/obstacles/ship1/LOSNode/u_d", cur_obs_speed)
    rospy.set_param("/obstacles/ship2/LOSNode/u_d", cur_obs_speed)


"""
Simple function that calculates the euclidean distance of the two
supplied arrays.
"""
def calculate_distance(fpos, spos):
    return distance.euclidean(fpos, spos)

main_position = (0, 0, 0)
obs1_position = (sys.maxsize, sys.maxsize, sys.maxsize)
obs2_position = (sys.maxsize, sys.maxsize, sys.maxsize)

min_obs1_distance = sys.maxsize
min_obs2_distance = sys.maxsize

"""
Convenience function to clean global state during iterations.
"""
def reset_global_values():
    global main_position
    global obs1_position
    global obs2_position

    global min_obs1_distance
    global min_obs2_distance

    main_position = (0, 0, 0)
    obs1_position = (sys.maxsize, sys.maxsize, sys.maxsize)
    obs2_position = (sys.maxsize, sys.maxsize, sys.maxsize)

    min_obs1_distance = sys.maxsize
    min_obs2_distance = sys.maxsize

"""
Callback that recomputes the new minimum distance values of the main ship with obs1 and
obs2 each time that main ship position changes.
"""
def main_pos_callback(data):
    global min_obs1_distance
    global min_obs2_distance

    main_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    actual_obs1_dist = calculate_distance(main_position, obs1_position)
    if (actual_obs1_dist < min_obs1_distance):
        min_obs1_distance = actual_obs1_dist

    actual_obs2_dist = calculate_distance(main_position, obs2_position)
    if (actual_obs2_dist < min_obs2_distance):
        min_obs2_distance = actual_obs2_dist

"""
Callback that recomputes the new minimum distance values of the main ship with obs1 each time
that obs1 position changes.
"""
def obs1_min_callback(data):
    global min_obs1_distance

    obs1_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    actual_obs1_dist = calculate_distance(main_position, obs1_position)
    if (actual_obs1_dist < min_obs1_distance):
        min_obs1_distance = actual_obs1_dist


"""
Callback that recomputes the new minimum distance values of the main ship with obs2 each time
that obs2 position changes.
"""
def obs2_pos_callback(data):
    global min_obs2_distance

    obs2_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    actual_obs2_dist = calculate_distance(main_position, obs2_position)
    if (actual_obs2_dist < min_obs2_distance):
        min_obs2_distance = actual_obs2_dist

"""
Function that encapsulate the subscriptions to movement topics.

:returns: An array with the current subscriber handlers.
"""
def subscribe_to_pos():
    # Subscribe to positions

    rospy.init_node("position_listener", anonymous=True)

    pos_listener = rospy.Subscriber("/asv/pose", PoseStamped, main_pos_callback)
    obs1_listener = rospy.Subscriber("/obstacles/ship1/pose", PoseStamped, obs1_min_callback)
    obs2_listener = rospy.Subscriber("/obstacles/ship2/pose", PoseStamped, obs2_pos_callback)

    return (pos_listener, obs1_listener, obs2_listener)

if __name__== "__main__":
    # Arrays to store the results of all the iterations
    mins_obs1 = []
    mins_obs2 = []
    # Sample time to check if the computation of the iterations is being done properly
    time = 30
    # File to store the results
    csv_filename = "results.csv"

    for cur_speed in s_params["cur_speed"]:
        for cur_ra in s_params["ra"]:
            for cur_tug_speed in s_params["tug_speed"]:
                for cur_obs_speed in s_params["obs_speed"]:
                    # Initialization of the currrent iteration
                    set_cur_params(cur_speed, cur_ra, cur_tug_speed, cur_obs_speed)
                    (ml, obs1_l, obs2_l) = subscribe_to_pos()
                    child = launch_simulation()

                    # Section to be replace with a proper ending condition
                    start = rospy.Time.now()
                    while not rospy.is_shutdown():
                        end = rospy.Time.now() - start > rospy.Duration.from_sec(time)
                        if end:
                            child.send_signal(signal.SIGINT)
                            child.terminate()
                            break

                    # Store the results of this iteration
                    mins_obs1.append(min_obs1_distance)
                    mins_obs2.append(min_obs2_distance)

                    # Write the header only if the file have been created by the first time
                    file_exists = os.path.isfile(csv_filename)

                    with open(csv_filename, mode='a') as csv_file:
                        fieldnames = ["cur_speed", "ra", "tug_speed", "obs_speed", "results"]
                        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                        if not file_exists:
                            writer.writeheader()

                        writer.writerow(
                            { 'cur_speed': cur_speed,
                              'ra': cur_ra,
                              'tug_speed': cur_tug_speed,
                              'obs_speed': cur_obs_speed,
                              'results': [min_obs1_distance , min_obs2_distance]
                            }
                        )

                    # Cleaning iteration values
                    reset_global_values()
                    ml.unregister()
                    obs1_l.unregister()
                    obs2_l.unregister()

                break
            break
        break

