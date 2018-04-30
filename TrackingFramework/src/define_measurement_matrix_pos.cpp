#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "kalman.hpp"

int define_measurement_matrix() {


  Eigen::MatrixXd C_lh(6, 9); // Measurement matrix C_lh : for light house measurement
  
  C_lh << 1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0,
	  0, 0, 0, 0, 0, 0, 1, 0, 0,
	  0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0,
	  0, 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::MatrixXd C_IMU(6, 9); // Measurement matrix C_IMU : for IMU measurement
  
  C_IMU << 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0, 0, 0, 0, 0, 0, 0, 0, 0,
	   0, 0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0, 0,
	   0, 0, 0, 0, 0, 0, 0, 0, 1;

}
