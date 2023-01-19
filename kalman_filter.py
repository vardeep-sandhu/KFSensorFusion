import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
    def __init__(self) -> None:
        pass

    # def setMetrices(self, x_0: np.array, P : np.array, A: np.array,\
    #                  Q: np.array, H_lidar: np.array, H_radar: np.array,\
    #                  R_radar: np.array, R_lidar: np.array) -> None:
    def setMetrices(self, x_0):
        # Setting different matricies
        self.P = np.array([
                    [0.025, 0, 0, 0],
                    [0, 0.025, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                    ])
        
        # Transition matrix
        self.A = np.array([
                [1.0, 0, 1.0, 0],               
                [0, 1.0, 0, 1.0],
                [0, 0, 1.0, 0],
                [0, 0, 0, 1.0]
                ])
                
        # Measurement matrix (LiDAR)
        self.H_lidar = np.array([ 
                [1.0, 0, 0, 0],					
                [0, 1.0, 0, 0]
                ])

        self.H_radar = np.identity(4)
        # Measurement noise covariance matrix (LiDAR)
        self.R_lidar = np.array([
                [1, 0],                     
                [0, 1]
                ])
        self.R_radar = np.identity(4)
        self.noise_ax = 10
        self.noise_ay = 10
        # Process noise covariance matrix
        self.Q = np.array([
                [0.25 * self.noise_ax, 0, 0.5 * self.noise_ax, 0],                
                [0, 0.25 * self.noise_ay, 0, 0.5 * self.noise_ax],
                [0.5 * self.noise_ax, 0, self.noise_ax, 0],
                [0, 0.5 * self.noise_ay, 0, self.noise_ay]
                ])
        self.x = x_0

        # self.P = P
        # self.A = A
        # self.H_lidar = H_lidar
        # self.H_radar = H_radar
        # self.R_lidar = R_lidar
        # self.R_radar = R_radar
        
        self.I = np.identity(4)
        # self.x = x_0
        # self.Q = Q
    
    def update_metrices(self, dt):
        dt = dt/1000_000
        dt_2 = dt * dt
        dt_3 = dt_2 * dt
        dt_4 = dt_3 * dt
        # Updation of matrix A with dt value
        self.A[0][2] = dt
        self.A[1][3] = dt
        # Updation of matrix Q  with dt value
        self.Q[0][0] = dt_4/4* self.noise_ax
        self.Q[0][2] = dt_3/2* self.noise_ax
        self.Q[1][1] = dt_4/4* self.noise_ay
        self.Q[1][3] = dt_3/2* self.noise_ay
        self.Q[2][0] = dt_3/2* self.noise_ax
        self.Q[2][2] = dt_2* self.noise_ax
        self.Q[3][1] = dt_3/2* self.noise_ay
        self.Q[3][3] = dt_2* self.noise_ay
        
    def predict(self) -> None:
        # Prediction Step
        self.x = self.A @ self.x
        self.P = (self.A @ self.P @ self.A.T) + self.Q
    
    def update(self, z: np.array, lidar: bool) -> None:
        # Update Step
        if lidar:
            H = self.H_lidar
            R = self.R_lidar
        else:
            H = self.H_radar
            R = self.R_radar
            
        Y = z - H @ self.x
        S = R + (H @ self.P @ H.T)
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ Y)
        self.P = (self.I - K @ H) @ self.P
        
    def getX(self) -> np.array:
        # Getting new state
        return self.x
