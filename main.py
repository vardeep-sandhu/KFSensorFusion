import numpy as np
import pickle
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter
import os
import utils 

def main():
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

    kf = KalmanFilter(x_0 = x_0)

    for i in range(1, len(measurements)):
        kf.predict()
        observation = utils.lidar_obj_to_arry(observations[i])
        prediction = kf.update(observation)
        predictions.append(prediction)
        print('iteration', i, 'x: ', predictions[i])
    
    predictions = np.concatenate(predictions).reshape(-1, 4)

    plt.plot(predictions[:,0], predictions[:,1], label = 'Kalman Filter Prediction')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()

    