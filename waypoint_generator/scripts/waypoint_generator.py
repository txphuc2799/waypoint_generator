#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped

import rospy
import ruamel.yaml


class WaypointsGenerator(object):

    def __init__(self, file_name):

        rospy.init_node("waypoints_generator")

        # Variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        self.frame_id = 'map'

        self.yaml = ruamel.yaml.YAML()
        self.yaml.default_flow_style = None

        self.file_name_ = file_name

        # Subscribers
        self.sub_amcl_pose_ = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.sub_amcl_pose_cb)


    def sub_amcl_pose_cb(self, pose: PoseWithCovarianceStamped):
        
        self.frame_id = pose.header.frame_id
        self.position_x = pose.pose.pose.position.x
        self.position_y = pose.pose.pose.position.y
        self.orientation_z = pose.pose.pose.orientation.z
        self.orientation_w = pose.pose.pose.orientation.w


    def save_waypoints(self):

        position_name = input("Position name: ")

        n = 1
        waypoints = []

        while not rospy.is_shutdown():
            
            _input = int(input("Enter position option (1: Only one, 0: More positions): "))

            if _input:
                pose_data = [float(f'{self.position_x:.6f}'), float(f'{self.position_y:.6f}'), 
                             float(f'{self.orientation_z:.6f}'), float(f'{self.orientation_w:.6f}')]

                with open(self.file_name_, 'r') as file:
                    data = self.yaml.load(file)

                data[f'{position_name}'] = {}
                data[f'{position_name}']['frame_id'] = self.frame_id
                data[f'{position_name}']['pose'] = pose_data

                print(data[f'{position_name}'])

                with open(self.file_name_, 'w') as f:
                    self.yaml.dump(data, f)
                    print("Written to file succesfully!")
                break

            elif _input == 0:
                while True:
                    
                    input_ = int(input(f"Get position {n}? (1: Yes, 0: No): "))

                    pose_data_ = [float(f'{self.position_x:.6f}'), float(f'{self.position_y:.6f}'), 
                                  float(f'{self.orientation_z:.6f}'), float(f'{self.orientation_w:.6f}')]

                    if input_:
                        waypoints.append(pose_data_)
                        print(f"Get position {n} successfully!")
                        n += 1
                        continue

                    else:
                        if n == 1:
                            rospy.logwarn("Get position error: No position data, try again...!")
                            continue

                        with open(self.file_name_, 'r') as file:
                            data = self.yaml.load(file)

                        data[f'{position_name}'] = {}
                        data[f'{position_name}']['frame_id'] = self.frame_id
                        data[f'{position_name}']['pose'] = waypoints

                        print(data[f'{position_name}'])

                        with open(self.file_name_, 'w') as f:
                            self.yaml.dump(data, f)
                            print(f"Written all waypoints to file succesfully!")
                        break
                break
            else:
                rospy.logwarn("Option is not incorrect, try again!")
                continue

if __name__ == "__main__":

    file_name = rospy.get_param("~file_name", "/waypoint_generator/cfg/waypoints.yaml") # Your directory 
    sub_amcl_pose = WaypointsGenerator(file_name)
    sub_amcl_pose.save_waypoints()
    rospy.loginfo("Write done!")

