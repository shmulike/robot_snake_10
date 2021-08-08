import numpy as np
import rosbag

bag = rosbag.Bag("~/Home/Desktop/bag/all.bag")

data = bag.read_messages(topics==['\robot_snake_10\joint_cmd'])

print(data)