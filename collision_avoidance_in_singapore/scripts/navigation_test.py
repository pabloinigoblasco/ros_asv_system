#!/usr/bin/env python
import copy
import csv
import os.path
import rospy
import subprocess
import signal
import scipy
import sys
import tf
import tf.transformations
import math
import numpy

# Import the datatype for the messages
from geometry_msgs.msg import PoseStamped

# Import the euclidean distance function
from scipy.spatial import distance

# Launch a new simulation

s_params = {
    # "cur_speed": [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4],
    # "ra": [10, 20, 30, 40, 50, 60, 70],
    # "obs_speed": [0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    # "tug_speed": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],

    "cur_speed": [0.2,0.8,1.4],
    "ra": [10, 40,70],
    "obs_speed": [0,3.0,6.0],
    "tug_speed": [1.0,4,7.0],
    "cur_speed": [0.2, 0.8, 1.4],

    # "cur_speed": [0.0],
    # "ra": [10],
    # "obs_speed": [3.0],
    # "tug_speed": [7.0],

    "one_obs": True
}

main_position = (0, 0, 0)
obs1_position = (sys.maxsize, sys.maxsize, sys.maxsize)
obs2_position = (sys.maxsize, sys.maxsize, sys.maxsize)

min_obs1_distance = sys.maxsize
min_obs2_distance = sys.maxsize

current_obs1_distance = sys.maxsize
current_obs2_distance = sys.maxsize

last_main_pose = None
last_obs1_pose = None
last_obs2_pose = None

overtake_right_1 = None
overtake_right_2 = None

goal_reached = False
goal_position = None
Ra = 0
goal_dist = 0


class HeadOnDistanceMetric1:
    """
    HEAD-ON Distance Metric 1
    """

    def __init__(self):
        self.distance = None
        self.time = None
        self.threshold = 0.98  # approx 12degress

    def update(self, ellapsed, main_ship_pose, other_ship_pose):
        if self.distance is None and main_ship_pose is not None and other_ship_pose is not None:
            orientation_main_ship = tf.transformations.euler_from_quaternion(
                [main_ship_pose.orientation.x, main_ship_pose.orientation.y, main_ship_pose.orientation.z, main_ship_pose.orientation.w])[2]
            orientation_other_ship = tf.transformations.euler_from_quaternion(
                [other_ship_pose.pose.orientation.x, other_ship_pose.pose.orientation.y, other_ship_pose.pose.orientation.z, other_ship_pose.pose.orientation.w])[2]

            main_ship_orientation_vector = [
                math.cos(orientation_main_ship), math.sin(orientation_main_ship)]
            orientation_other_ship_vector = [
                math.cos(orientation_other_ship), math.sin(orientation_other_ship)]

            dotproduct = main_ship_orientation_vector[0] * orientation_other_ship_vector[0] + \
                main_ship_orientation_vector[1] * \
                orientation_other_ship_vector[1]

            value = math.fabs(dotproduct)
            rospy.logwarn("metric1 value: " + str(value))
            if value < self.threshold:
                # triggers
                self.distance = calculate_distance_pose(
                    main_ship_pose, other_ship_pose.pose)
                self.time = ellapsed
                rospy.logerr("metric1 TRIGGERs")


class HeadOnDistanceMetric3:
    """
    HEAD-ON Distance Metric 3
    """

    def __init__(self):
        self.distance = None
        self.time = None
        self.threshold = 0.98  # approx 12degress
        self.goalposition = rospy.get_param("/asv/LOSNode/waypoints")[-1]
        self.original_trajectory_vector = None

    def update(self, ellapsed, main_ship_pose, other_ship_pose):
        if self.distance is None and main_ship_pose is not None and other_ship_pose is not None:

            joinvector = [other_ship_pose.pose.position.x - main_ship_pose.position.x,
                          other_ship_pose.pose.position.y - main_ship_pose.position.y]
            norm = numpy.linalg.norm(joinvector)
            joinvector = joinvector/norm

            if self.original_trajectory_vector is None:
                self.original_trajectory_vector = [
                    self.goalposition[0] - main_ship_pose.position.x, self.goalposition[1] - main_ship_pose.position.y]
                norm = numpy.linalg.norm(self.original_trajectory_vector)
                self.original_trajectory_vector = self.original_trajectory_vector/norm

            # dotproduct
            value = joinvector[0]*self.original_trajectory_vector[0] + \
                joinvector[1]*self.original_trajectory_vector[1]

            rospy.logwarn(self.goalposition)
            rospy.logwarn("metric3 value: " + str(value))
            if value < self.threshold:
                # triggers
                self.distance = calculate_distance_pose(
                    main_ship_pose, other_ship_pose.pose)
                self.time = ellapsed
                rospy.logerr("metric3 TRIGGERs")


def launch_simulation(simulation_id):
    """
    Function to launch a new simulation.

    :returns: A handler of the launched child process.
    """

    # the two_obstacles_head.launch includes the single head-on.launch as a particular case
    child = subprocess.Popen(
        ["roslaunch", "collision_avoidance_in_singapore", "two_obstacles_head.launch"])
    #child_screen_record = subprocess.Popen(["ffmpeg", "-video_size", "1024x768", "-framerate 5", "-f", "x11grab", "-i" ,":0.0+100,200", str(simulation_id) + ".mp4"])
    cmd = "/usr/bin/ffmpeg -video_size 1920x1080 -framerate 5 -f x11grab -i "+os.environ['DISPLAY']+" " + \
        str(i)+".ogv"
    rospy.logwarn(cmd)
    child_screen_record = subprocess.Popen(cmd.split(" "))
    # child.wait() #You can use this line to block the parent process untill the child process finished.
    # Printing process information
    print("parent process")
    print(child.poll())

    rospy.loginfo('The PID of child: %d', child.pid)
    print("The PID of child:", child.pid)

    return child, child_screen_record


def set_cur_params(cur_speed, cur_ra, cur_tug_speed, cur_obs_speed):
    """
    Helper function to set current parameters to be used during one iteration.
    """
    # Setting current speed
    rospy.set_param("/asv/asv/Vx_current", cur_speed)
    rospy.set_param("/obstacles/ship1/ship1/Vx_current", cur_speed)
    rospy.set_param("/obstacles/ship2/ship2/Vx_current", cur_speed)

    # Setting acceptance radius
    rospy.set_param("/asv/LOSNode/acceptance_radius", cur_ra)
    rospy.set_param("/obstacles/ship1/LOSNode/acceptance_radius", cur_ra)
    rospy.set_param("/obstacles/ship2/LOSNode/acceptance_radius", cur_ra)

    # Setting tug speed
    rospy.set_param("/asv/LOSNode/u_d", cur_tug_speed)

    # Setting obs speed
    rospy.set_param("/obstacles/ship1/LOSNode/u_d", cur_obs_speed)
    rospy.set_param("/obstacles/ship2/LOSNode/u_d", cur_obs_speed)


def calculate_distance_pose(pose1, pose2):
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y

    return math.sqrt(dx*dx + dy*dy)


def calculate_distance(fpos, spos):
    """
    Simple function that calculates the euclidean distance of the two
    supplied arrays.
    """

    return distance.euclidean(fpos, spos)


def reset_global_values():
    """
    Convenience function to clean global state during iterations.
    """

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
    last_main_pose = None

    overtake_right_1 = None
    overtake_right_2 = None

    goal_reached = False
    goal_dist = 0


def update_min_distance_1(main_ship_position, obs1_position):
    global min_obs1_distance
    actual_obs1_dist = calculate_distance(main_ship_position, obs1_position)
    if (actual_obs1_dist < min_obs1_distance):
        min_obs1_distance = actual_obs1_dist

    rospy.logwarn_throttle(
        1, "obs1 callback, min dist: " + str(min_obs1_distance))


def update_min_distance_2(main_ship_position, obs2_position):
    global min_obs2_distance
    actual_obs2_dist = calculate_distance(main_ship_position, obs2_position)
    if (actual_obs2_dist < min_obs2_distance):
        min_obs2_distance = actual_obs2_dist

    rospy.logwarn_throttle(
        1, "obs2 callback, min dist: " + str(min_obs2_distance))


def update_overtake_side_1(main_ship_pose, last_obs1_pose):
    global overtake_right_1

    # on overtake edge 1 - only happens once
    if overtake_right_1 is None and not last_obs1_pose is None and main_ship_pose.pose.position.y > last_obs1_pose.pose.position.y:
        if main_ship_pose.pose.position.x > last_obs1_pose.pose.position.x:
            overtake_right_1 = True
        else:
            overtake_right_1 = False


def update_overtake_side_2(main_ship_pose, last_obs2_pose):
    global overtake_right_2

    # on overtake edge 2 - only happens once
    if overtake_right_2 is None and not last_obs2_pose is None and main_ship_pose.pose.position.y > last_obs2_pose.pose.position.y:
        if main_ship_pose.pose.position.x > last_obs2_pose.pose.position.x:
            overtake_right_2 = True
        else:
            overtake_right_2 = False


def update_goal_reached(main_position):
    global goal_position
    global Ra
    global goal_reached

    if not goal_position is None:
        goal_dist = calculate_distance(goal_position, main_position[0:2])
        if goal_dist < Ra*2:
            goal_reached = True


def main_pos_callback(msg):
    """
    Callback that recomputes the new minimum distance values of the main ship with obs1 and
    obs2 each time that main ship position changes.
    """
    global current_obs1_distance
    global current_obs2_distance

    global min_obs1_distance
    global min_obs2_distance

    global last_obs1_pose
    global last_obs2_pose
    global last_main_pose

    global overtake_right_2
    global goal_reached
    global goal_dist
    global obs2_position
    global obs1_position
    global main_position

    last_main_pose = copy.deepcopy(msg.pose)
    main_position = (msg.pose.position.x,
                     msg.pose.position.y, msg.pose.position.z)
    update_min_distance_1(main_position, obs1_position)
    update_min_distance_2(main_position, obs2_position)

    update_overtake_side_1(msg, last_obs1_pose)
    update_overtake_side_2(msg, last_obs2_pose)

    update_goal_reached(main_position)


def obs1_min_callback(msg):
    """
    Callback that recomputes the new minimum distance values of the main ship with obs1 each time
    that obs1 position changes.
    """
    global obs1_position
    global main_position
    global last_obs1_pose

    last_obs1_pose = copy.deepcopy(msg)
    obs1_position = (msg.pose.position.x,
                     msg.pose.position.y, msg.pose.position.z)
    update_min_distance_1(main_position, obs1_position)


def obs2_pos_callback(msg):
    """
    Callback that recomputes the new minimum distance values of the main ship with obs2 each time
    that obs2 position changes.
    """
    global obs2_position
    global main_position
    global last_obs2_pose

    last_obs2_pose = copy.deepcopy(msg)
    obs2_position = (msg.pose.position.x,
                     msg.pose.position.y, msg.pose.position.z)
    update_min_distance_2(main_position, obs2_position)


def subscribe_to_pos():
    """
    Function that encapsulate the subscriptions to movement topics.

    :returns: An array with the current subscriber handlers.
    """

    # Subscribe to positions

    pos_listener = rospy.Subscriber(
        "/asv/pose", PoseStamped, main_pos_callback)
    obs1_listener = rospy.Subscriber(
        "/obstacles/ship1/pose", PoseStamped, obs1_min_callback)
    obs2_listener = rospy.Subscriber(
        "/obstacles/ship2/pose", PoseStamped, obs2_pos_callback)

    return (pos_listener, obs1_listener, obs2_listener)


def check_overtaketime(one_obs, overtake_right_1, overtake_right_2):
    """
    Check overtime timer start depending on the number of obstacles
    """
    if one_obs:
        return not overtake_right_1 is None or not overtake_right_2 is None
    else:
        return not overtake_right_1 is None and not overtake_right_2 is None


if __name__ == "__main__":
    rospy.init_node("position_listener", anonymous=True)
    headon_distance_metric1_obs1 = None
    headon_distance_metric3_obs1 = None

    # Sample time to check if the computation of the iterations is being done properly
    timeout = 450
    timeout_after_overtake = 8
    # File to store the results
    csv_filename = "results.csv"

    i = 0
    rospy.sleep(5)
    skip_count = 6
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

                    rospy.logwarn(
                        "SIMULATION {i} curr_speed: {cur_speed}, ra: {cur_ra}, tug_speed: {cur_tug_speed}, obs_speed: {cur_obs_speed})".format(**locals()))
                    i += 1
                    if i < skip_count:
                        continue

                    rospy.sleep(5)

                    # Initialization of the currrent iteration
                    set_cur_params(cur_speed, cur_ra,
                                   cur_tug_speed, cur_obs_speed)

                    if not rospy.is_shutdown():
                        rospy.sleep(3)

                    child, child_screen_record = launch_simulation(i)

                    rospy.sleep(5)
                    goal_position = rospy.get_param(
                        "/asv/LOSNode/waypoints")[-1]

                    headon_distance_metric1_obs1 = HeadOnDistanceMetric1()
                    headon_distance_metric3_obs1 = HeadOnDistanceMetric3()

                    # Section to be replace with a proper ending condition
                    start = rospy.Time.now()
                    overtaketime = None
                    while not rospy.is_shutdown():

                        rospy.loginfo(".")
                        # end if timeout
                        ellapsed = rospy.Time.now() - start
                        end = ellapsed > rospy.Duration.from_sec(timeout)

                        rospy.loginfo("simulation ellapsed : " + str(ellapsed.to_sec()) +
                                      ", goal dist: " + str(goal_dist) + " Ra: "+str(Ra))

                        # end if passed too much time after overtaking
                        if overtaketime is None and check_overtaketime(s_params["one_obs"], overtake_right_1, overtake_right_2):
                            overtaketime = rospy.Time.now()

                        if not overtaketime is None:
                            ellapsed_from_overtake = rospy.Time.now() - overtaketime
                            rospy.loginfo(
                                "ellapsed from overtake: " + str(ellapsed_from_overtake.to_sec()))
                            end_overtake = ellapsed_from_overtake > rospy.Duration.from_sec(
                                timeout_after_overtake)
                            end = end or end_overtake

                        headon_distance_metric1_obs1.update(ellapsed,
                                                            last_main_pose, last_obs1_pose)
                        headon_distance_metric3_obs1.update(ellapsed,
                                                            last_main_pose, last_obs1_pose)

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
                            'headon_distance_obstacle_1': headon_distance_metric1_obs1.distance,
                            'headon_time_obstacle_1': headon_distance_metric1_obs1.time.to_sec(),
                            'headon_distance_obstacle_3': headon_distance_metric3_obs1.distance,
                            'headon_time_obstacle_3': headon_distance_metric3_obs1.time.to_sec(),
                            'avoiding_side_1': "right" if overtake_right_1 else "left",

                            # 'overtake_right_1': overtake_right_1,
                            # 'overtake_right_2': overtake_right_2,
                            "sim_duration": (rospy.Time.now() - start).to_sec()
                        }

                        rospy.loginfo("current result : " + str(result))

                        rospy.sleep(1)

                    # Write the header only if the file have been created by the first time
                    file_exists = os.path.isfile(csv_filename)

                    with open(csv_filename, mode='a') as csv_file:
                        fieldnames = ["simulation_id",
                                      "cur_speed",
                                      "ra",
                                      "tug_speed",
                                      "obs_speed",
                                      "min_obstacle_distance_1",
                                      "min_obstacle_distance_2",
                                      "min_obstacle_distance",
                                      "headon_distance_obstacle_1",
                                      "headon_time_obstacle_1",
                                      "headon_distance_obstacle_3",
                                      "headon_time_obstacle_3",
                                      "avoiding_side_1",
                                      # "overtake_right_1",
                                      # "overtake_right_2",
                                      "sim_duration"]

                        writer = csv.DictWriter(
                            csv_file, fieldnames=fieldnames)

                        if not file_exists:
                            writer.writeheader()

                        writer.writerow(result)

                    rospy.sleep(5)

                    # Cleaning iteration values
                    reset_global_values()