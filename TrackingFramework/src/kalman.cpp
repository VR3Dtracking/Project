/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
                           double dt,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& C,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R,
                           const Eigen::MatrixXd& P)
: A(A), C(C), Q(Q), R(R), P0(P),
m(C.rows()), n(A.rows()), dt(dt), initialized(false),
I(n, n), x_hat(n), x_hat_new(n)
{
    I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
    x_hat = x0;
    //P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
}

void KalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {
    
    if(!initialized)
        throw std::runtime_error("Filter is not initialized!");
    
    x_hat_new = A * x_hat;
    P = A*P*A.transpose() + Q;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    x_hat_new += K * (y - C*x_hat_new);
    P = (I - K*C)*P;
    x_hat = x_hat_new;
    
    t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd C, const Eigen::MatrixXd Q) {
    
    this->A = A;
    this->C = C;
    this->Q = Q;
    this->dt = dt;
    update(y);
}

void KalmanFilter::fusionUp( double dt, const Eigen::VectorXd& y,const Eigen::MatrixXd& C,const Eigen::MatrixXd& R) {
    
    //this->R=R;
    int n = 9; // Number of states
    int m = 3; // Number of measurements
    
    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    
    // Update A
    A << 1, dt, dt*dt/2, 0, 0, 0, 0, 0, 0,
         0, 1, dt, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, dt, dt*dt/2, 0, 0, 0,
         0, 0, 0, 0, 1, dt, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, dt, dt*dt/2,
         0, 0, 0, 0, 0, 0, 0, 1, dt,
         0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    // Update Q
    Q << pow(dt,6)/ 36, pow(dt,5) / 12, pow(dt,4) / 6,pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt, 6) / 36, pow(dt, 5) / 12, pow(dt,4) / 6,
    pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt, 5)/ 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3,
    pow (dt,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt ,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt, 4) / 6, pow(dt,3) / 2, pow(dt,2),
    pow(dt,6) / 36,pow( dt,5) / 12, pow(dt,4) / 6, pow(dt, 6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6,
    pow(dt, 5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3,
    pow(dt,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt ,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt,4) / 6, pow(dt,3) / 2, pow(dt, 2),
    pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6,
    pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3,
    pow(dt,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt, 4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt,4) / 6, pow(dt,3) / 2, pow(dt, 2);
    
    
    // Update the filter
    update(y,dt,A,C,Q);
}

void KalmanFilter::fusionInit(double dt, const Eigen::VectorXd& y,const Eigen::MatrixXd& C,const Eigen::VectorXd& x) {
    
    int n = 9; // Number of states
    int m = 3; // Number of measurements
    
    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance
    Eigen::MatrixXd I(n, n); // Estimate error covariance
    // Free object
    A << 1, dt, dt*dt/2, 0, 0, 0, 0, 0, 0,
    0, 1, dt, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, dt, dt*dt/2, 0, 0, 0,
    0, 0, 0, 0, 1, dt, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, dt, dt*dt/2,
    0, 0, 0, 0, 0, 0, 0, 1, dt,
    0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    // Covariance matrices
    Q << pow(dt,6)/ 36, pow(dt,5) / 12, pow(dt,4) / 6,pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt, 6) / 36, pow(dt, 5) / 12, pow(dt,4) / 6,
    pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt, 5)/ 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3,
    pow (dt,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt ,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt, 4) / 6, pow(dt,3) / 2, pow(dt,2),
    pow(dt,6) / 36,pow( dt,5) / 12, pow(dt,4) / 6, pow(dt, 6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6,
    pow(dt, 5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3,
    pow(dt,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt ,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt,4) / 6, pow(dt,3) / 2, pow(dt, 2),
    pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6, pow(dt,6) / 36, pow(dt,5) / 12, pow(dt,4) / 6,
    pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3, pow(dt,5) / 12, pow(dt,4) / 4, pow(dt,3) / 3,
    pow(dt,4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt, 4) / 6, pow(dt,3) / 2, pow(dt,2), pow(dt,4) / 6, pow(dt,3) / 2, pow(dt, 2);
    
    R << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;

    
    P << 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    I<< 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1;
    
    // Construct the filter
    this->dt=dt;
    this->A=A;
    this->C=C;
    this->Q=Q;
    this->R=R;
    this->P=P;
    this->I=I;

    init(0,x);
}
