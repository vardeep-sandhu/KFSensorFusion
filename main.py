import numpy as np
import pickle
import matplotlib.pyplot as plt
import os
import utils 
import sys
from radar_lidar_fusion import LiDAR_object, Radar_object, Ground_Truth
import argparse


def parse_commandline():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend','-b', type=str, required= True, choices=['cpp', 'python'])
    args = parser.parse_args()
    return args

def main():
    args = parse_commandline()
    basedir = "data"
    print("Loading RADAR, LiDAR and GT")
    lidar_path = os.path.join(basedir, "lidar.bin")
    assert os.path.isfile(lidar_path), "LiDAR path is incorrect"
    
    lidar_file = open(lidar_path, "rb")
    lidar_states = pickle.load(lidar_file)
    
    radar_path = os.path.join(basedir, "radar.bin")
    assert os.path.isfile(radar_path), "RADAR path is incorrect"

    radar_file = open(radar_path, "rb")
    radar_states = pickle.load(radar_file)
    
    gt_path = os.path.join(basedir, "ground_truth.bin")
    assert os.path.isfile(gt_path), "GT path is incorrect"
    
    ground_truth_file = open(gt_path, "rb")
    ground_states = pickle.load(ground_truth_file)

    # sanity check 
    assert len(lidar_states) == len(radar_states) == len(ground_states), "Error in the dataset"
    print("*" * 80)
    print("LiDAR and RADAR observation have length:", len(lidar_states))

    # measurements = radar_states
    # observations = lidar_states
    
    x_0 = utils.radar_obj_to_arry(radar_states[0])
    predictions = [x_0]

    # State covariance matrix
    P = np.array([
                [0.025, 0, 0, 0],
                [0, 0.025, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
                ])
    # Transition matrix
    A = np.array([
            [1.0, 0, 1.0, 0],               
            [0, 1.0, 0, 1.0],
            [0, 0, 1.0, 0],
            [0, 0, 0, 1.0]
            ])
            
    # Measurement matrix (LiDAR)
    H_lidar = np.array([ 
            [1.0, 0, 0, 0],					
            [0, 1.0, 0, 0]
            ])

    H_radar = np.identity(4)
    # Measurement noise covariance matrix (LiDAR)
    R_lidar = np.array([
            [1, 0],                     
            [0, 1]
            ])
    R_radar = np.identity(4)
    noise_ax = 10
    noise_ay = 10
    # Process noise covariance matrix
    Q = np.array([
            [0.25 * noise_ax, 0, 0.5 * noise_ax, 0],                
            [0, 0.25 * noise_ay, 0, 0.5 * noise_ax],
            [0.5 * noise_ax, 0, noise_ax, 0],
            [0, 0.5 * noise_ay, 0, noise_ay]
            ])

    if args.backend == "cpp":
        print("*" * 80)
        print("Loading CPP implementation of Kalman Filter")
        sys.path.append("build/")
        from kalmanfilter import KalmanFilter
    elif args.backend == "python":
        print("*" * 80)
        print("Loading Python implementation of Kalman Filter")
        from kalman_filter import KalmanFilter

    kf = KalmanFilter()
    kf.setMetrices(x_0, P, A, Q, H_lidar, H_radar, R_radar, R_lidar)

    for i in range(1, len(radar_states)):
        # Prediction step
        kf.predict()
        # Alternate radar and lidar measurement step
        print("*" * 50)
        print("Time stamp of measurement ", i)
        if i%2 == 1:
            measurement = utils.radar_obj_to_arry(radar_states[i])
            print("Making update using RADAR measurement: ", measurement)
            kf.update(measurement, lidar=False)
        elif i%2 == 0:
            measurement = utils.lidar_obj_to_arry(lidar_states[i])
            print("Making update using LiDAR measurement: ", measurement)
            kf.update(measurement, lidar=True)
        else:
            print("not used")
        prediction = kf.getX()
        print("Prediction is: ", prediction)
        predictions.append(prediction)
    
    predictions = np.concatenate(predictions).reshape(-1, 4)
    ADE = utils.ade_calculate(predictions, ground_states)
    
    print("*" * 80)
    print("The Average Displacement Error (ADE) is: ", ADE)
    print("*" * 80)
    print("The plot is saved in the current working dir.")
    print("*" * 80)

    utils.save_plot(predictions, ground_states)

if __name__ == "__main__":
    main()