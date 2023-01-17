import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
    def __init__(self) -> None:
        pass

    def setMetrices(self, x_0: np.array, P : np.array, A: np.array,\
                     Q: np.array, H_lidar: np.array, H_radar: np.array,\
                     R_radar: np.array, R_lidar: np.array) -> None:
        # Setting different matricies
        self.P = P
        self.A = A
        self.H_lidar = H_lidar
        self.H_radar = H_radar
        self.R_lidar = R_lidar
        self.R_radar = R_radar
        
        self.I = np.identity(4)
        self.x = x_0
        self.Q = Q
        
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
