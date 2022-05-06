#!/usr/bin/env python

import glob
import os
import sys
import carla
import rospy
import numpy as np

from math import sin, radians, sqrt
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
    initial_time = last_time = rospy.Time.now()
   
    starting_waypoint = map.get_waypoint(location, project_to_road=True)
    starting_waypoint_coordinate = np.array([starting_waypoint.transform.location.x, starting_waypoint.transform.location.x, 0])
    starting_lane_id = starting_waypoint.lane_id

    left_lane_id = starting_waypoint.get_left_lane().lane_id
    right_lane_id = starting_waypoint.get_right_lane().lane_id
    
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        d_time = current_time - last_time
        last_time = current_time
        current_location = actors[0].get_location()
        current_location.z = 0.0
        nearest_waypoint = map.get_waypoint(current_location, project_to_road=True)


        nearest_waypoint_coordinate = np.array([nearest_waypoint.transform.location.x,
                                                nearest_waypoint.transform.location.y,
                                                0])
        next_waypoint_coordinate = np.array([nearest_waypoint.next(0.01)[0].transform.location.x, 
                                             nearest_waypoint.next(0.01)[0].transform.location.y,
                                             0])
        current_location_coordinate = np.array([current_location.x, current_location.y, 0])             

        next_wp_vector = next_waypoint_coordinate - nearest_waypoint_coordinate
        nearest_wp_heading_vector = current_location_coordinate - nearest_waypoint_coordinate
        is_left_or_right = np.dot(np.cross(next_wp_vector, nearest_wp_heading_vector), np.array([0,0,1]))
        
        
        if nearest_waypoint.lane_id == starting_lane_id:
            distance = nearest_waypoint.transform.location.distance(current_location)
            if is_left_or_right < 0: distance *= -1      
        elif nearest_waypoint.lane_id >= right_lane_id:
            distance = nearest_waypoint.get_left_lane().transform.location.distance(current_location)
        elif nearest_waypoint.lane_id <= right_lane_id:
            distance = -nearest_waypoint.get_right_lane().transform.location.distance(current_location)

        np.dot(next_wp_vector, nearest_wp_heading_vector)/np.linalg.norm(next_wp_vector)
        np.linalg.norm(next_wp_vector)

        
        print(nearest_waypoint.next(0.01)[0].transform.location)
        print(np.linalg.norm(nearest_waypoint_coordinate))
        print(current_location)
        print(distance)
        print(left_lane_id, nearest_waypoint.lane_id, right_lane_id)
        print(d_time.to_sec())
        print((current_time - initial_time).to_sec())
        # distance between lanes are 3.5m 

        
        world.wait_for_tick()
            

        # host_waypoint = waypoint
        # route_distance = 2
        # previous_waypoint = host_waypoint.previous(route_distance)[0]
        # next_waypoint = host_waypoint.next(route_distance)[0]
        # _transform = next_waypoint.transform
        # _location, _rotation  = _transform.location, _transform.rotation
        # x1, y1 = _location.x, _location.y
        # yaw1 = _rotation.yaw

        # _transform = previous_waypoint.transform
        # _location, _rotation  = _transform.location, _transform.rotation
        # x2, y2 = _location.x, _location.y
        # yaw2 = _rotation.yaw

        # c = 2*sin(radians((yaw1-yaw2)/2)) / sqrt((x1-x2)**2 + (y1-y2)**2)
        # print(c)




if __name__ == '__main__':

    main()
