import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
	def __init__(self, x_0 = None):

		self.x = x_0
		print(self.x)
		self.P = np.array([
				[0.25, 0, 0, 0],
				[0, 0.25, 0, 0],               # State Cov mat
				[0, 0, 0.01, 0],
				[0, 0, 0, 0.01]
				])
		self.A = np.array([
				[1.0, 0, 1.0, 0],               # Kinematic Equation 
				[0, 1.0, 0, 1.0],
				[0, 0, 1.0, 0],
				[0, 0, 0, 1.0]
				])
		self.H = np.array([ 
				[1.0, 0, 0, 0],					#Mapping of state to measurement 
				[0, 1.0, 0, 0]
				])
		self.I = np.identity(4)

		self.R = np.array([
				[5, 0],                        # Measurement Cov mat
				[0, 5]
				])
		noise_ax = 10
		noise_ay = 10

		self.Q = np.array([
				[0.25 * noise_ax, 0, 0.5 * noise_ax, 0],               # Kinematic Equation 
				[0, 0.25 * noise_ay, 0, 0.5 * noise_ax],
				[0.5 * noise_ax, 0, noise_ax, 0],
				[0, 0.5 * noise_ay, 0, noise_ay]
				])

	def predict(self):
		# Prediction Step
		self.x = self.A @ self.x
		self.P = (self.A @ self.P @ self.A.T) + self.Q
	
	def update(self, z):
		Y = z - self.H @ self.x
		S = self.R + (self.H @ self.P @ self.H.T)
		K = self.P @ self.H.T @ np.linalg.inv(S)

		self.x = self.x + (K @ Y) 				# updated state 
		self.P = (self.I - K @ self.H @ self.P)  @ (self.I - K @ self.H).T + (K @ self.R) @ K.T
		return self.x

if __name__ == "__main__":
	measurements = np.load("radar.npy")
	observations = np.load("lidar.npy")
	predictions = [measurements[0]]

	gt = np.load("ground.npy")

	x_0 = measurements[0]
	measurements = measurements[1: , :]

	ground_truth = gt[0]

	kf = KalmanFilter(x_0 = x_0)

	for i in range (len(measurements)):
		kf.predict()
		predictions.append(kf.update(observations[i+1]))
		print('iteration', i, 'x: ', predictions[i])
		
	predictions = np.concatenate(predictions).reshape(-1, 4)

	plt.plot(measurements[:,0], measurements[:,1], label = 'radar')
	plt.plot(observations[:,0], observations[:,1], label = 'lidar')

	plt.plot(predictions[:,0], predictions[:,1], label = 'Kalman Filter Prediction')
	plt.plot(gt[:,0], gt[:,1], label = 'GT')

	plt.legend()
	plt.show()
