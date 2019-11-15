#!/usr/bin/env python

import threading
import rospy
import actionlib
import queue
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
import time


# Define configuration variables here
MAX_QUEUE_SIZE = 20  # Number of waypoints to store
waypoint_topic = "/mir_fleet_manager/waypoint_goal"
cancel_topic = "/mir_fleet_manager/reset"
viz_topic = "mir_fleet_manager/waypoints"
frame_id = "map"
waypoints = []  # Empty initial waypoints queue


def wait_for_waypoint(viz_publisher):
    # Function to get move_to_waypoint messages
    global waypoints
    last_timestamp = 0
    while True:
        data = rospy.wait_for_message(waypoint_topic, PoseStamped)
        msg_timestamp = data.header.stamp.secs
        if msg_timestamp <= last_timestamp:
            continue
        else:
            rospy.loginfo('Recieved waypoint message')
            waypoints.append(data)
            update_viz(viz_publisher)
            last_timestamp = msg_timestamp
            time.sleep(0.5)


def wait_for_reset(viz_publisher):
    # Function to get move_to_waypoint messages
    global waypoints
    while True:
        data = rospy.wait_for_message(cancel_topic, PointStamped)
        if data:
            rospy.loginfo('Recieved reset signal')
            waypoints = []
            clear(viz_publisher)
            update_viz(viz_publisher)
            time.sleep(0.5)


def clear(viz_publisher):
    all_poses = PoseArray()
    all_poses.header.frame_id = frame_id
    viz_publisher.publish(all_poses)


def update_viz(viz_publisher):
    global waypoints

    if waypoints:
        rospy.loginfo("There are " + str(len(waypoints)) + "waypoints left.")
        all_poses = PoseArray()
        all_poses.header = waypoints[-1].header
        all_poses.poses = [pose.pose for pose in waypoints]
        viz_publisher.publish(all_poses)


def execute(client):
    global waypoints
    # Execute waypoints each in sequence
    while waypoints:
        waypoint = waypoints[0]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = waypoint.header.frame_id
        goal.target_pose.pose.position = waypoint.pose.position
        goal.target_pose.pose.orientation = waypoint.pose.orientation
        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                      (waypoint.pose.position.x, waypoint.pose.position.y))
        client.send_goal(goal)
        client.wait_for_result()
        if waypoints:
            waypoints.pop(0)
        rospy.loginfo("There are " + str(len(waypoints)) + "waypoints left.")


def main():
    global waypoints
    rospy.init_node('mir_fleet_manager')
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Connect to vendor controller interface
    rospy.loginfo('Connecting to move_base...')
    client.wait_for_server()
    rospy.loginfo('Connected to move_base.')

    # Visualizer for markers
    viz_publisher = rospy.Publisher(viz_topic, PoseArray, queue_size=20)

    # Create interface for accepting new waypoints as part of a plan
    waypoint_thread = threading.Thread(
        target=lambda: wait_for_waypoint(viz_publisher))
    rospy.loginfo(
        "Waiting to recieve waypoints via PoseStamped msg on topic %s" % waypoint_topic)
    waypoint_thread.start()

    # Listen for reset commands
    reset_thread = threading.Thread(
        target=lambda: wait_for_reset(viz_publisher))
    reset_thread.start()

    while not rospy.is_shutdown():
        update_viz(viz_publisher)
        if waypoints:
            execute(client)

        time.sleep(1)

    waypoint_thread.join()
    reset_thread.join()

    # Create interface for cancelling a plan
