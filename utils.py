import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def lidar_obj_to_arry(obj):
    arr = np.array([
			obj.pose_x, obj.pose_y 
			])
    return arr 


def radar_obj_to_arry(obj):
    arr = np.array([
			obj.pose_x, obj.pose_y, obj.vel_x, obj.vel_y  
			])
    return arr 

def ade_calculate(pred, gt):
    gt = np.array([radar_obj_to_arry(i) for i in gt])
    sq_error = (pred[:, :2] - gt[:, :2])**2
    sqrt_diff = np.sqrt(sq_error[:, 0] + sq_error[:, 1])
    
    ADE = np.mean(sqrt_diff)
    return ADE

def save_plot(predictions, gt):
    sns.set()
    gt = np.array([radar_obj_to_arry(i) for i in gt])
    plt.plot(predictions[:,0], predictions[:,1], label = 'Kalman Filter Prediction')
    plt.plot(gt[:,0], gt[:,1], label = 'GT', linestyle='--', color='r')
    plt.xlabel('x-direction')
    plt.ylabel('y-direction') 
    plt.legend()
    plt.savefig('resulting_trajectories.png')
