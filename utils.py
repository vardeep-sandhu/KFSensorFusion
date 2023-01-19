import sys
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def lidar_obj_to_arry(obj):
    arr = np.array([
			obj.pose_x, obj.pose_y
			])
    return obj.timestamp, arr 


def radar_obj_to_arry(obj):
    arr = np.array([
			obj.pose_x, obj.pose_y, obj.vel_x, obj.vel_y  
			])
    return obj.timestamp, arr 

def ade_calculate(pred, gt):
    gt = np.array([radar_obj_to_arry(i)[1] for i in gt])
    sq_error = (pred[:, :2] - gt[:, :2])**2
    sqrt_diff = np.sqrt(sq_error[:, 0] + sq_error[:, 1])
    
    ADE = np.mean(sqrt_diff)
    return ADE

def save_plot(predictions, gt):
    sns.set()
    gt = np.array([radar_obj_to_arry(i)[1] for i in gt])
    plt.plot(predictions[:,0], predictions[:,1], label = 'Kalman Filter Prediction')
    plt.plot(gt[:,0], gt[:,1], label = 'GT', linestyle='--', color='r')
    plt.xlabel('x-direction')
    plt.ylabel('y-direction') 
    plt.legend()
    plt.savefig('resulting_trajectories.png')

def rmse_vel(pred, gt):
    gt = np.array([radar_obj_to_arry(i) for i in gt])
    sq_error = (pred[:, 2:] - gt[:, 2:])**2
    sqrt_diff = np.sqrt(sq_error[:, 0] + sq_error[:, 1])
    
    rmse = np.mean(sqrt_diff)
    return rmse

def CalculateRMSE(estimations, ground_truth):
    rmse = np.zeros([4, 1])
    ground_truth = np.array([radar_obj_to_arry(i)[1] for i in ground_truth])
    
    rmse[0][0] =  np.sqrt(((estimations[:, 0] - ground_truth[:, 0]) ** 2).mean())

    rmse[1][0] =  np.sqrt(((estimations[:, 1] - ground_truth[:, 1]) ** 2).mean())
    rmse[2][0] =  np.sqrt(((estimations[:, 2] - ground_truth[:, 2]) ** 2).mean())
    rmse[3][0] =  np.sqrt(((estimations[:, 3] - ground_truth[:, 3]) ** 2).mean())
    return rmse
