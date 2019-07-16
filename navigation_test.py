#!/usr/bin/env python

import rospy
import subprocess
import signal
import scipy

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



main_positions = []
obs1_positions = []
obs2_positions = []

def main_pos_callback(data):
    main_positions.append((data.pose.position.x, data.pose.position.y, data.pose.position.z))

def obs1_pos_callback(data):
    obs1_positions.append((data.pose.position.x, data.pose.position.y, data.pose.position.z))

def obs2_pos_callback(data):
    obs2_positions.append((data.pose.position.x, data.pose.position.y, data.pose.position.z))

def calculate_distances(fpos, spos):
    return list(map(distance.euclidean, fpos, spos))

def subscribe_to_pos():
    # Subscribe to positions

    rospy.init_node("position_listener", anonymous=True)
    rospy.Subscriber("/asv/pose", PoseStamped, main_pos_callback)
    rospy.Subscriber("/obstacles/ship1/pose", PoseStamped, obs1_pos_callback)
    rospy.Subscriber("/obstacles/ship2/pose", PoseStamped, obs2_pos_callback)
    # rospy.spin()

if __name__== "__main__":
    for cur_speed in s_params["cur_speed"]:
        for cur_ra in s_params["ra"]:
            for cur_tug_speed in s_params["tug_speed"]:
                for cur_obs_speed in s_params["obs_speed"]:
                    child = launch_simulation()
                    # set_cur_params(cur_speed, cur_ra, cur_tug_speed, cur_obs_speed)
                    subscribe_to_pos()

                    rospy.sleep(15)

                    min_obs1 = min(calculate_distances(main_positions, obs1_positions))
                    min_obs2 = min(calculate_distances(main_positions, obs2_positions))

                    print("Minimum distance Obs1: " + str(min_obs1))
                    print("Minimum distance Obs2: " + str(min_obs2))

                    # child.wait()

                    child.send_signal(signal.SIGINT) #You may also use .terminate() method
                    child.terminate()

                    break
                break

            break
        break
