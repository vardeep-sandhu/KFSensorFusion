import numpy as np

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