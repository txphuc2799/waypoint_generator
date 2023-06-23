#!/usr/bin/env python3

import yaml
import rospy

class ReadYaml():

    def __init__(self):
        
        self.b = self.read_position('b',0)
        print("a = ", self.b)


    def read_position(self, postion_name: str, order: int):
        """
        order = 1 -> Only one
        order = 0 -> More positions
        """
        pos = []
        with open("/home/phuc/catkin_ws/src/turtlebot3/turtlebot3_controller/cfg/turtlebot3_waypoints.yaml", 'r') as file:
            data = yaml.safe_load(file)

            if order:
                position = data[f'{postion_name}']['pose']['position']
                orientation = data[f'{postion_name}']['pose']['orientation']

                # position = list(position.values())
                # orientation = list(orientation.values())
                # pos.append(position)
                # pos.append(orientation)
            else:
                position = data[f'{postion_name}']['pose']['position'][0]
                position = list(position.values())
            return position


if __name__ == '__main__':
    rospy.init_node("read_yaml_file")
    a = ReadYaml()