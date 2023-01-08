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
    lidar_path = os.path.join(basedir, "lidar.bin")
    assert os.path.isfile(lidar_path), "LiDAR path is incorrect"
    
    lidar_file = open(lidar_path, "rb")
    lidar_states = pickle.load(lidar_file)
    
    radar_path = os.path.join(basedir, "radar.bin")
    assert os.path.isfile(radar_path), "RADAR path is incorrect"

    radar_file = open(radar_path, "rb")
    radar_states = pickle.load(radar_file)
    
    gt_path = os.path.join(basedir, "radar.bin")
    assert os.path.isfile(gt_path), "GT path is incorrect"
    
    ground_truth_file = open(gt_path, "rb")
    ground_states = pickle.load(ground_truth_file)

    # sanity check 
    assert len(lidar_states) == len(radar_states) == len(ground_states), "Error in the dataset"

    measurements = radar_states
    observations = lidar_states
    
    x_0 = utils.radar_obj_to_arry(measurements[0])
    predictions = [x_0]

    P = np.array([
                [0.25, 0, 0, 0],
                [0, 0.25, 0, 0],               # State Cov mat
                [0, 0, 0.01, 0],
                [0, 0, 0, 0.01]
                ])
    A = np.array([
            [1.0, 0, 1.0, 0],               # Kinematic Equation 
            [0, 1.0, 0, 1.0],
            [0, 0, 1.0, 0],
            [0, 0, 0, 1.0]
            ])
    H = np.array([ 
            [1.0, 0, 0, 0],					#Mapping of state to measurement 
            [0, 1.0, 0, 0]
            ])
    R = np.array([
            [5, 0],                        # Measurement Cov mat
            [0, 5]
            ])
    noise_ax = 10
    noise_ay = 10

    Q = np.array([
            [0.25 * noise_ax, 0, 0.5 * noise_ax, 0],               # Kinematic Equation 
            [0, 0.25 * noise_ay, 0, 0.5 * noise_ax],
            [0.5 * noise_ax, 0, noise_ax, 0],
            [0, 0.5 * noise_ay, 0, noise_ay]
            ])

    if args.backend == "cpp":
        # change name for cpp lib 
        sys.path.append("build/")
        from kalmanfilter import KalmanFilter
    elif args.backend == "python":
        from kalman_filter import KalmanFilter

    kf = KalmanFilter()
    kf.setMetrices(x_0, P, A, Q, R, H)

    for i in range(1, len(measurements)):
        kf.predict()
        
        observation = utils.lidar_obj_to_arry(observations[i])
        kf.update(observation)

        prediction = kf.getX()
        predictions.append(prediction)
        print('iteration', i, 'x: ', predictions[i])
    
    predictions = np.concatenate(predictions).reshape(-1, 4)

    plt.plot(predictions[:,0], predictions[:,1], label = 'Kalman Filter Prediction')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()

    