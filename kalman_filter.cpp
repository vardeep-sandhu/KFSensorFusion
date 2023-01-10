#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter{

  private:
    int noise_ax = 10;
    int noise_ay = 10;
    
    VectorXd x;

    MatrixXd P;

    MatrixXd A;

    MatrixXd H;
    MatrixXd I;
    
    MatrixXd R;
    MatrixXd Q;

  public:
    KalmanFilter(){};
    void setMetrices(const VectorXd& x_0, const MatrixXd& P, const MatrixXd& A, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& H);
    VectorXd getX() const;
    void predict();
    void update(const VectorXd& z);
};

void KalmanFilter::setMetrices(
  // Setting Matricies
  const VectorXd& x_0, const MatrixXd& P, const MatrixXd& A, const MatrixXd& Q, const MatrixXd& R, const MatrixXd& H){

  this->noise_ax = noise_ax;
  this->noise_ay = noise_ay;
  
  this->H = H;
  this->R = R;

  this->I = MatrixXd::Identity(4, 4);
  this->x = x_0;
  this->P = P;
  this->A = A;
  this->Q = Q;
}

VectorXd KalmanFilter::getX() const{
  return x;
}

// Prediction Step 
void KalmanFilter::predict(){
  x = A * x;
  P = A * P * A.transpose() + Q;
}

// Measurement and Update Step
void KalmanFilter::update(const VectorXd& z){

  VectorXd Y = z - H * x;

  MatrixXd S = R + (H * P * H.transpose());
  
  MatrixXd K = P * H.transpose() * S.inverse();
  
  x = x + (K * Y);
  
  P = (I - K * H) * P;
  }
