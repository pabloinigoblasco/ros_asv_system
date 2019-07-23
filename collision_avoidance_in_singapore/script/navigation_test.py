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
def launch_simulation(simulation_id):
    child = subprocess.Popen(["roslaunch","collision_avoidance_in_singapore","two_obstacles_head.launch"])
    #child_screen_record = subprocess.Popen(["ffmpeg", "-video_size", "1024x768", "-framerate 5", "-f", "x11grab", "-i" ,":0.0+100,200", str(simulation_id) + ".mp4"])
    cmd = "/usr/bin/ffmpeg -video_size 1920x1080 -framerate 5 -f x11grab -i :0.0 " + str(i)+".ogv"
    rospy.logwarn(cmd)
    child_screen_record = subprocess.Popen(cmd.split(" "))
    #child.wait() #You can use this line to block the parent process untill the child process finished.
    # Printing process information
    print("parent process")
    print(child.poll())

    rospy.loginfo('The PID of child: %d', child.pid)
    print ("The PID of child:", child.pid)

    return child, child_screen_record

"""
Helper function to set current parameters to be used during one iteration.
"""
def set_cur_params(cur_speed, cur_ra, cur_tug_speed, cur_obs_speed):
    # Setting current speed
    rospy.set_param("/asv/asv/Vx_current", cur_speed)
    rospy.set_param("/obstacles/ship1/ship1/Vx_current", cur_speed)
    rospy.set_param("/obstacles/ship2/ship2/Vx_current", cur_speed)

    # Setting acceptance radius
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

current_obs1_distance = sys.maxsize
current_obs2_distance = sys.maxsize

last_obs1_pose = None
last_obs2_pose = None

overtake_right_1 = None
overtake_right_2 = None

goal_reached = False
goal_position = None
Ra = 0
goal_dist = 0 

"""
Convenience function to clean global state during iterations.
"""
def reset_global_values():
    global current_obs1_distance
    global current_obs2_distance
    global main_position
    global obs1_position
    global obs2_position

    global min_obs1_distance
    global min_obs2_distance

    global last_obs1_pose
    global last_obs2_pose

    global overtake_right_1 
    global overtake_right_2
    global goal_reached
    global goal_dist
     
    main_position = (0, 0, 0)
    obs1_position = (sys.maxsize, sys.maxsize, sys.maxsize)
    obs2_position = (sys.maxsize, sys.maxsize, sys.maxsize)

    min_obs1_distance = sys.maxsize
    min_obs2_distance = sys.maxsize

    current_obs1_distance = sys.maxsize
    current_obs2_distance = sys.maxsize

    last_obs1_pose = None
    last_obs2_pose = None

    overtake_right_1 = None
    overtake_right_2 = None

    goal_reached=False
    goal_dist = 0

"""
Callback that recomputes the new minimum distance values of the main ship with obs1 and
obs2 each time that main ship position changes.
"""
def main_pos_callback(data):
    global current_obs1_distance
    global current_obs2_distance
    global min_obs1_distance
    global min_obs2_distance
    global last_obs1_pose
    global last_obs2_pose
    global overtake_right_1
    global overtake_right_2
    global goal_reached
    global goal_dist
    global obs2_position
    global obs1_position

    main_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    actual_obs1_dist = calculate_distance(main_position, obs1_position)
    if (actual_obs1_dist < min_obs1_distance):
        min_obs1_distance = actual_obs1_dist

    actual_obs2_dist = calculate_distance(main_position, obs2_position)
    if (actual_obs2_dist < min_obs2_distance):
        min_obs2_distance = actual_obs2_dist

    #on overtake edge 1 - only happens once
    if overtake_right_1 is None and not last_obs1_pose is  None and data.pose.position.y > last_obs1_pose.pose.position.y:
        if data.pose.position.x > last_obs1_pose.pose.position.x:
            overtake_right_1 = True
        else:
            overtake_right_1 = False

    #on overtake edge 2 - only happens once
    if  overtake_right_2 is None and not last_obs2_pose is None and data.pose.position.y > last_obs2_pose.pose.position.y:
        if data.pose.position.x > last_obs2_pose.pose.position.x:
            overtake_right_2 = True
        else:
            overtake_right_2 = False

    if not goal_position is None:
       goal_dist = calculate_distance(goal_position, main_position[0:2]) 
       if goal_dist < Ra*2:
           goal_reached = True

"""
Callback that recomputes the new minimum distance values of the main ship with obs1 each time
that obs1 position changes.
"""
def obs1_min_callback(data):
    global min_obs1_distance
    global last_obs1_pose
    global obs1_position
    
    last_obs1_pose = data
    obs1_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    actual_obs1_dist = calculate_distance(main_position, obs1_position)
    if (actual_obs1_dist < min_obs1_distance):
        min_obs1_distance = actual_obs1_dist

    rospy.logwarn_throttle(1, "obs1 callback")


"""
Callback that recomputes the new minimum distance values of the main ship with obs2 each time
that obs2 position changes.
"""
def obs2_pos_callback(data):
    global min_obs2_distance
    global last_obs2_pose
    global obs2_position

    last_obs2_pose = data
    obs2_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

    actual_obs2_dist = calculate_distance(main_position, obs2_position)
    if (actual_obs2_dist < min_obs2_distance):
        min_obs2_distance = actual_obs2_dist
    
    
    rospy.logwarn_throttle(1, "obs1 callback2")

"""
Function that encapsulate the subscriptions to movement topics.

:returns: An array with the current subscriber handlers.
"""
def subscribe_to_pos():
    # Subscribe to positions

    pos_listener = rospy.Subscriber("/asv/pose", PoseStamped, main_pos_callback)
    obs1_listener = rospy.Subscriber("/obstacles/ship1/pose", PoseStamped, obs1_min_callback)
    obs2_listener = rospy.Subscriber("/obstacles/ship2/pose", PoseStamped, obs2_pos_callback)

    return (pos_listener, obs1_listener, obs2_listener)

if __name__== "__main__":
    rospy.init_node("position_listener", anonymous=True)

    # Sample time to check if the computation of the iterations is being done properly
    timeout = 450
    timeout_after_overtake = 8
    # File to store the results
    csv_filename = "results.csv"

    i = 0 
    rospy.sleep(5)
    skip_count = 292
    # We use the same instance of ROSCore during all the simulation, there is no
    # need to subscribe again in each iteration.
    #
    # WIP: Unregistering may have been the cause of subscribers not receiving topics
    # messages properly.
    (ml, obs1_l, obs2_l) = subscribe_to_pos()

    for cur_speed in s_params["cur_speed"]:
        for cur_ra in s_params["ra"]:
            Ra = cur_ra
            for cur_tug_speed in s_params["tug_speed"]:
                for cur_obs_speed in s_params["obs_speed"]:
                    if rospy.is_shutdown():
                        break

                    rospy.logwarn("SIMULATION {i} curr_speed: {cur_speed}, ra: {cur_ra}, tug_speed: {cur_tug_speed}, obs_speed: {cur_obs_speed})".format(**locals()))
                    i+=1             
                    if i < skip_count:
                        continue

                    rospy.sleep(5)

                    # Initialization of the currrent iteration
                    set_cur_params(cur_speed, cur_ra, cur_tug_speed, cur_obs_speed)
                    child, child_screen_record = launch_simulation(i)

                    rospy.sleep(2)
                    goal_position = rospy.get_param("/asv/LOSNode/waypoints")[-1]

                    # Section to be replace with a proper ending condition
                    start = rospy.Time.now()
                    overtaketime = None
                    while not rospy.is_shutdown():
                        
                        # end if timeout
                        ellapsed = rospy.Time.now() - start
                        end = ellapsed > rospy.Duration.from_sec(timeout)
                        
                        rospy.loginfo("simulation ellapsed : " + str(ellapsed.to_sec())+ ", goal dist: "+ str(goal_dist)+ " Ra: "+str(Ra))
                            
                        # end if passed too much time after overtaking
                        if overtaketime is None and not overtake_right_1 is None and not overtake_right_2 is None:
                            overtaketime = rospy.Time.now()

                        if not overtaketime is None:
                            ellapsed_from_overtake = rospy.Time.now() - overtaketime
                            rospy.loginfo("ellapsed from overtake: "+ str(ellapsed_from_overtake.to_sec()))
                            end_overtake =  ellapsed_from_overtake > rospy.Duration.from_sec(timeout_after_overtake)
                            end = end or end_overtake

                        if end or goal_reached:
                            try:
                                child.send_signal(signal.SIGINT)
                                child.send_signal(signal.SIGINT)
                            except:
                                pass
                            try:
                                child_screen_record.send_signal(signal.SIGINT)
                                child_screen_record.send_signal(signal.SIGINT)
                            except:
                                pass
                            try:
                                child.terminate()
                                child.terminate()
                            except:
                                pass
                            try:
                                child_screen_record.terminate()
                                child_screen_record.terminate()
                            except:
                                pass

                            break

                        result = { 
                              'simulation_id': i,
                              'cur_speed': cur_speed,
                              'ra': cur_ra,
                              'tug_speed': cur_tug_speed,
                              'obs_speed': cur_obs_speed,
                              'min_obstacle_distance_1': min_obs1_distance,
                              'min_obstacle_distance_2': min_obs2_distance,
                              'min_obstacle_distance': min([min_obs1_distance, min_obs2_distance]),
                              'overtake_right_1': overtake_right_1,
                              'overtake_right_2': overtake_right_2,
                              "sim_duration": (rospy.Time.now() - start).to_sec()
                            }

                        rospy.loginfo("current result : " + str(result))

                        rospy.sleep(1)



                    # Write the header only if the file have been created by the first time
                    file_exists = os.path.isfile(csv_filename)

                    with open(csv_filename, mode='a') as csv_file:
                        fieldnames = ["simulation_id", "cur_speed", "ra", "tug_speed", "obs_speed", "min_obstacle_distance_1", "min_obstacle_distance_2", "min_obstacle_distance", "overtake_right_1", "overtake_right_2", "sim_duration"]
                        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                        if not file_exists:
                            writer.writeheader()

                        writer.writerow(result
                        )

                    rospy.sleep(5)

                    # Cleaning iteration values
                    reset_global_values()
