/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
 #ifndef MODULES_ESTIMATORS_QEKF_H
 #define MODULES_ESTIMATORS_QEKF_H
 
 #include "Parameters.h"
 
 #include <chrono>
 #include <Eigen/Dense>
 
 class QEKF
 {
     public:
         QEKF(Parameters& params);
         ~QEKF();
 
         void Reset();
         void Reset(const Eigen::Vector3f accelerometer);
         void Reset(const Eigen::Vector3f accelerometer, const float heading);
         void Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope);
         void Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const bool EstimateBias);
         void Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const float heading, const bool EstimateBias);
         void Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const bool EstimateBias, const float dt);
         void Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const float heading, const bool EstimateBias, const float dt);
         void Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const float heading, const bool UseHeadingForCorrection, const bool SensorDriven, const bool EstimateBias, const bool EstimateYawBias, const bool CreateQdotFromDifference, const float cov_acc[9], const float cov_gyro[9], const float GyroscopeTrustFactor, const float sigma2_omega, const float sigma2_heading, const float sigma2_bias, const bool AccelerometerVibrationDetectionEnabled, const float AccelerometerVibrationNormLPFtau, const float AccelerometerVibrationCovarianceVaryFactor, const float AccelerometerCovarianceMaxVaryFactor, const float g, const float dt);
 
         void GetQuaternion(Eigen::Quaternionf& q);
         void GetQuaternionDerivative(Eigen::Vector4f& dq_);
         void GetGyroBias(float bias[3]);
         void GetQuaternionCovariance(Eigen::Matrix<float, 4, 4>& Cov_q);
         void GetQuaternionDerivativeCovariance(float Cov_dq[4*4]);
         void GetAngularVelocityCovariance(float Cov_omega[3*3]);
         void GetBiasCovariance(float Cov_bias[3*3]);
 
     private:
         Parameters& _params;
         std::chrono::_V2::high_resolution_clock::time_point _prevTimerValue;
 
         /* State estimate */
         float X[10];    // state estimates = { q[0], q[1], q[2], q[3], omega_x, omega_y, omega_z, gyro_bias[0], gyro_bias[1], gyro_bias[2] }
         float P[10*10]; // covariance matrix
 };
     
     
 #endif
 