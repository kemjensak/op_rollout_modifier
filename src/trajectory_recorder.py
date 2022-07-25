#!/usr/bin/env python

import carla
import rospy
from std_msgs.msg import Bool
import numpy as np
from datetime import datetime
import csv

from math import sin, radians, sqrt
# distance between lanes are 3.5m 
distance_between_lane = 3.5/2

class CarlaTrajectoryRecorder:
    def __init__(self):
        
        rospy.Subscriber("aes_flag", Bool, self.callback_aes_flag)

        self.last_x_distance = 0
        self.is_started_recording = False
        self.is_AES_activated = False
        self.AES_uprising_time = None
        self.AES_activated_time = None
        self.cutout_time_of_lv = None

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        
        self.world = self.client.get_world()

        self.world.wait_for_tick()

        self.map = self.world.get_map()

        self.vut_actor = self.world.get_actors().filter('vehicle.toyota.*')
        self.lv_actor = self.world.get_actors().filter('vehicle.lincoln.*')
        self.vut_actor[0].show_debug_telemetry(enabled=True)
        self.initial_location = self.vut_actor[0].get_location()
        self.initial_location.z = 0.0
        self.initial_time = rospy.Time.now()
        
        self.starting_wp = self.map.get_waypoint(self.initial_location, project_to_road=True)
        
        self.initial_location_coordinate = np.array([-9.55792141, -162.67843628, 0])
        # [  -9.55792141 -162.67843628    0.        ] is start of linear part in test scenario
        
        self.starting_lane_id = self.starting_wp.lane_id

        self.left_lane_id = self.starting_wp.get_left_lane().lane_id
        self.right_lane_id = self.starting_wp.get_right_lane().lane_id

        
        

        filename = datetime.now().strftime("%Y-%m-%d-%H%M%S")
        self.f = open('/home/irol/ros_ws/catkin_ws/src/carla_sim/op_rollout_modifier/export/write_'+ filename +'.csv','a')
        self.wr = csv.writer(self.f)

        # print("Recording on file: %s" % self.client.start_recorder('/home/irol/ros_ws/catkin_ws/src/carla_sim/op_rollout_modifier/export/write_'+ filename +'.log'))



    def callback_aes_flag(self, msg):
        self.is_AES_activated = msg.data
        self.AES_activated_time = rospy.Time.now()

    def exit(self):
        self.f.close()
        # self.client.stop_recorder()
        print('exit')

    def get_xy_distance(self):

        current_location = self.vut_actor[0].get_location()
        current_location.z = 0.0
        nearest_wp = self.map.get_waypoint(current_location, project_to_road=True)

        if nearest_wp.lane_id == self.starting_lane_id:
            pass    
        elif nearest_wp.lane_id >= self.right_lane_id:
            nearest_wp = nearest_wp.get_left_lane()
        elif nearest_wp.lane_id <= self.right_lane_id:
            nearest_wp = nearest_wp.get_right_lane()


        nearest_wp_coordinate = np.array([nearest_wp.transform.location.x,
                                        nearest_wp.transform.location.y,
                                        0])
        next_wp_coordinate = np.array([nearest_wp.next(0.01)[0].transform.location.x, 
                                    nearest_wp.next(0.01)[0].transform.location.y,
                                    0])
        current_location_coordinate = np.array([current_location.x, current_location.y, 0])             

        next_wp_vector = next_wp_coordinate - nearest_wp_coordinate
        current_location_vector = current_location_coordinate - nearest_wp_coordinate

        projected_nearest_wp_vector = (next_wp_vector * np.dot(next_wp_vector, current_location_vector)/
                                    pow(np.linalg.norm(next_wp_vector),2))
        projected_nearest_wp_coordinate = nearest_wp_coordinate + projected_nearest_wp_vector

        is_left_or_right = np.dot(np.cross(next_wp_vector, current_location_vector), np.array([0,0,1]))

        x_distance = np.linalg.norm(projected_nearest_wp_coordinate - self.initial_location_coordinate)
        y_distance = np.linalg.norm(current_location_vector - projected_nearest_wp_vector)

        if is_left_or_right > 0: y_distance *= -1

        print(x_distance)
        print(y_distance)
        return(x_distance, y_distance)



    def main(self):

        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.initial_time).to_sec()

        x_distance, y_distance = self.get_xy_distance()

        if self.cutout_time_of_lv == None and self.map.get_waypoint(self.lv_actor[1].get_location(), project_to_road=True).lane_id == self.left_lane_id:
            self.cutout_time_of_lv = rospy.Time.now() # saves time when the LV's nearest waypoint changes to adjacent lane
        if self.AES_uprising_time == None and self.AES_activated_time != None and self.cutout_time_of_lv != None :
            self.AES_uprising_time = (self.AES_activated_time - self.cutout_time_of_lv).to_sec()
            print("recorded uprising time")
            print(self.map.get_waypoint(self.lv_actor[1].get_location(), project_to_road=True).lane_id)
        # print(current_location_coordinate)
        if self.is_started_recording == False and x_distance < 10: self.is_started_recording = True
            
        if self.is_started_recording == True and 380 > x_distance > 150 and self.AES_uprising_time != None:
            self.wr.writerow([elapsed_time, x_distance, y_distance, self.AES_uprising_time])
            print("recording now!")
        elif self.is_started_recording == True and 380 > x_distance > 150 and self.AES_uprising_time == None:
            self.wr.writerow([elapsed_time, x_distance, y_distance])
            print("recording now!")

        # phyx_con = self.vut_actor[0].get_physics_control()
        # print(phyx_con.wheels[3])
        
        self.world.wait_for_tick()


        

if __name__ == '__main__':
    rospy.init_node("trajectory_recorder")
    RunTest = CarlaTrajectoryRecorder()
    while not rospy.is_shutdown():
        RunTest.main()
    RunTest.exit()
    
    
