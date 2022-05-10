#!/usr/bin/env python

import glob
import os
import sys
import carla
import rospy
import numpy as np
from datetime import datetime
import csv

from math import sin, radians, sqrt
# distance between lanes are 3.5m 
distance_between_lane = 3.5/2

def main():

    rospy.init_node("trajectory_recorder")
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    
    world = client.get_world()

    world.wait_for_tick()
    map = world.get_map()
    actors = world.get_actors().filter('vehicle.toyota.*')

    location = actors[0].get_location()
    location.z = 0.0
    initial_time = last_time = rospy.Time.now()
    now = datetime.now()
    starting_waypoint = map.get_waypoint(location, project_to_road=True)
    starting_waypoint_coordinate = np.array([starting_waypoint.transform.location.x, starting_waypoint.transform.location.y, 0])
    starting_lane_id = starting_waypoint.lane_id

    left_lane_id = starting_waypoint.get_left_lane().lane_id
    right_lane_id = starting_waypoint.get_right_lane().lane_id
    filename = datetime.now().strftime("%Y-%m-%d-%H%M%S")
    f = open('/home/irol/ros_ws/catkin_ws/src/carla_sim/op_rollout_modifier/export/write_'+ filename +'.csv','a')
    wr = csv.writer(f)
    
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        elapsed_time = (current_time - initial_time).to_sec()
        current_location = actors[0].get_location()
        current_location.z = 0.0
        nearest_waypoint = map.get_waypoint(current_location, project_to_road=True)

        if nearest_waypoint.lane_id == starting_lane_id:
           pass    
        elif nearest_waypoint.lane_id >= right_lane_id:
            nearest_waypoint = nearest_waypoint.get_left_lane()
        elif nearest_waypoint.lane_id <= right_lane_id:
            nearest_waypoint = nearest_waypoint.get_right_lane()


        nearest_waypoint_coordinate = np.array([nearest_waypoint.transform.location.x,
                                                nearest_waypoint.transform.location.y,
                                                0])
        next_waypoint_coordinate = np.array([nearest_waypoint.next(0.01)[0].transform.location.x, 
                                             nearest_waypoint.next(0.01)[0].transform.location.y,
                                             0])
        current_location_coordinate = np.array([current_location.x, current_location.y, 0])             

        next_wp_vector = next_waypoint_coordinate - nearest_waypoint_coordinate
        nearest_wp_heading_vector = current_location_coordinate - nearest_waypoint_coordinate
        projected_nearest_wp_vector = (next_wp_vector * np.dot(next_wp_vector, nearest_wp_heading_vector)/
                                       pow(np.linalg.norm(next_wp_vector),2))
        projected_nearest_waypoint_coordinate = nearest_waypoint_coordinate + projected_nearest_wp_vector

        is_left_or_right = np.dot(np.cross(next_wp_vector, nearest_wp_heading_vector), np.array([0,0,1]))

        x_distance = np.linalg.norm(projected_nearest_waypoint_coordinate - starting_waypoint_coordinate)
        # y_distance = nearest_waypoint.transform.location.distance(current_location)
        y_distance = np.linalg.norm(nearest_wp_heading_vector - projected_nearest_wp_vector)
        if is_left_or_right > 0: y_distance *= -1

        print(x_distance)
        print(y_distance)
        # print(left_lane_id, nearest_waypoint.lane_id, right_lane_id)
        # print((current_time - initial_time).to_sec())
        # distance between lanes are 3.5m 
        if x_distance > 360:
            x_distance -= 360
            wr.writerow([elapsed_time, x_distance, y_distance])
        
        world.wait_for_tick()
        
    f.close()




if __name__ == '__main__':

    main()
    
