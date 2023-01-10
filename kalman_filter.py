import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
    def __init__(self) -> None:
        pass

    def setMetrices(self, x_0: np.array, P : np.array, A: np.array,\
                     Q: np.array , R: np.array, H: np.array) -> None:
        # Setting different matricies
        self.P = P
        self.A = A
        self.H = H
        self.I = np.identity(4)
        self.x = x_0
        self.R = R
        self.Q = Q
        
    def predict(self) -> None:
        # Prediction Step
        self.x = self.A @ self.x
        self.P = (self.A @ self.P @ self.A.T) + self.Q
    
    def update(self, z: np.array) -> None:
        # Update Step
        Y = z - self.H @ self.x
        S = self.R + (self.H @ self.P @ self.H.T)
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ Y)
        self.P = (self.I - K @ self.H) @ self.P
        
    def getX(self) -> np.array:
        # Getting new state
        return self.x
