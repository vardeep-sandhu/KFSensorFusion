import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
    def __init__(self):
        pass

    def setMetrices(self, x_0, P , A, Q, R, H):
        self.P = P
        self.A = A
        self.H = H
        self.I = np.identity(4)
        self.x = x_0
        self.R = R
        self.Q = Q
        
    def predict(self):
        # Prediction Step
        self.x = self.A @ self.x
        self.P = (self.A @ self.P @ self.A.T) + self.Q
    
    def update(self, z):
        Y = z - self.H @ self.x
        S = self.R + (self.H @ self.P @ self.H.T)
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ Y) 				# updated state 
        self.P = (self.I - K @ self.H) @ self.P
        #   @ (self.I - K @ self.H).T + (K @ self.R) @ K.T
        
    def getX(self):
        return self.x
