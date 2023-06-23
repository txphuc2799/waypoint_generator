#!/usr/bin/env python3

import yaml
import rospy

class ReadYaml():

    def __init__(self):
        
        self.b = self.read_position('v',0)
        print("a = ", self.b)


    def read_position(self, postion_name: str, order: int):
        """
        order = 1 -> Only one
        order = 0 -> More positions
        """
        pos = []
        with open("/waypoint_generator/cfg/waypoints.yaml", 'r') as file:
            data = yaml.safe_load(file)

            if order:
                position = data[f'{postion_name}']['pose'][0]
            else:
                position = data[f'{postion_name}']['pose']
            return position


if __name__ == '__main__':
    rospy.init_node("read_yaml_file")
    a = ReadYaml()
