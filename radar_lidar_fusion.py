# !/usr/bin/env python3

import numpy as np
import pickle
import matplotlib.pyplot as plt

def plot_traj(data):
    plt.plot(data[:, 0], data[:, 1]) 
    plt.show()
    pass 

class LiDAR_object:
    def __init__(self, timestamp, pose_x, pose_y):
        self.timestamp = timestamp
        self.pose_x = pose_x
        self.pose_y = pose_y

class Radar_object:
    def __init__(self, timestamp, pose_x, pose_y, vel_x, vel_y):
        self.timestamp = timestamp
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.vel_x = vel_x
        self.vel_y = vel_y
class Ground_Truth:
    def __init__(self, timestamp, pose_x, pose_y, vel_x, vel_y):
        self.timestamp = timestamp
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.vel_x = vel_x
        self.vel_y = vel_y

np.set_printoptions(suppress=True)


if __name__ == "__main__":
    lidar_file = open("data/lidar.bin", "rb")
    lidar_states = pickle.load(lidar_file)
    lidar_np = np.zeros((len(lidar_states), 2))
    for i in range(len(lidar_states)):
        pose_x = lidar_states[i].pose_x
        lidar_np[i,0] = pose_x 
        pose_y = lidar_states[i].pose_y
        lidar_np[i,1] = pose_y

    radar_file = open("data/radar.bin", "rb")
    radar_states = pickle.load(radar_file)
    radar_np = np.zeros((len(lidar_states), 4))
    for j in range(len(radar_states)):
        pose_x = radar_states[j].pose_x
        pose_y = radar_states[j].pose_y
        vel_x = radar_states[j].vel_x
        vel_y = radar_states[j].vel_y
        radar_np[j,0] = pose_x 
        radar_np[j,1] = pose_y
        radar_np[j,2] = vel_x 
        radar_np[j,3] = vel_y 
        
    ground_truth_file = open("data/ground_truth.bin", "rb")
    ground_states = pickle.load(ground_truth_file)
    ground_np = np.zeros((len(lidar_states), 4))
    for k in range(len(ground_states)):
        pose_x = ground_states[k].pose_x
        pose_y = ground_states[k].pose_y
        vel_x = ground_states[k].vel_x
        vel_y = ground_states[k].vel_y
        ground_np[k,0] = pose_x 
        ground_np[k,1] = pose_y
        ground_np[k,2] = vel_x 
        ground_np[k,3] = vel_y 
        
    np.save("radar.npy", radar_np)
    np.save("lidar.npy", lidar_np)
    np.save("ground.npy", ground_np)

    print(radar_np[:10])
