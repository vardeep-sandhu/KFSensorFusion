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

def load_data(path: str):
    assert os.path.isfile(path), "Path is incorrect"
    file = open(path, "rb")
    data = pickle.load(file)
    return data

def main():
    args = parse_commandline()
    basedir = "data"
    print("Loading RADAR, LiDAR and GT")

    lidar_path = os.path.join(basedir, "lidar.bin")
    radar_path = os.path.join(basedir, "radar.bin")
    gt_path = os.path.join(basedir, "ground_truth.bin")
    
    lidar_states = load_data(lidar_path)
    radar_states = load_data(radar_path)
    gt_states = load_data(gt_path)
    
    # sanity check 
    assert len(lidar_states) == len(radar_states) == len(gt_states), "Error in the dataset"
    print("*" * 80)
    print("LiDAR and RADAR observation have length:", len(lidar_states))

    prev_time, x_0 = utils.radar_obj_to_arry(radar_states[0])
    predictions = [x_0]

    # State covariance matrix
    
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
    kf.setMetrices(x_0)

    for i in range(1, len(radar_states)):
        # Alternate radar and lidar measurement step
        print("*" * 50)
        print("Time stamp of measurement ", i)
        if i%2 == 1:
            curr_time, measurement = utils.radar_obj_to_arry(radar_states[i])
            dt = curr_time - prev_time
            
            kf.update_metrices(dt)
            kf.predict()

            print("Making update using RADAR measurement: ", measurement)
            print(dt)
            kf.update(measurement, lidar=False)
            prev_time = curr_time
        
        elif i%2 == 0:
            curr_time, measurement = utils.lidar_obj_to_arry(lidar_states[i])
            dt = curr_time - prev_time
            kf.update_metrices(dt)
            kf.predict()
            
            print("Making update using LiDAR measurement: ", measurement)
            kf.update(measurement, lidar=True)
            prev_time = curr_time
        else:
            print("not used")
        prediction = kf.getX()
        print("Prediction is: ", prediction)
        predictions.append(prediction)
    
    predictions = np.concatenate(predictions).reshape(-1, 4)
    ADE = utils.ade_calculate(predictions, gt_states)
    rmse_vel = utils.CalculateRMSE(predictions, gt_states)
    
    print("*" * 80)
    print("The Average Displacement Error (ADE) is: ", ADE)
    print("*" * 80)
    print("The RMSE for velocity component is: ")
    print(rmse_vel)
    print("*" * 80)
    print("The plot is saved in the current working dir.")
    print("*" * 80)

    utils.save_plot(predictions, gt_states)

if __name__ == "__main__":
    main()