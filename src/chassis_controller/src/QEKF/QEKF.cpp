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

 #include "QEKF.h"

 #include <math.h>
 #include <string.h>  // for memcpy
 
 #include <Eigen/Dense>
 #include <cmath>
 
 #include "MathLib.h"
 #include "MathLib.h"  // for matrix symmetrization
 #include "QEKF_coder.h"
 #include "QEKF_initialize.h"
 #include "Quaternion.h"
 // #include "arm_math.h"
 #include "chassis_driver/math_utils.hpp"
 
 QEKF::QEKF(Parameters& params) : _params(params) { Reset(); }
 
 QEKF::~QEKF() {}
 
 void QEKF::Reset() {
     QEKF_initialize(_params.estimator.QEKF_P_init_diagonal, X, P);
 
     _prevTimerValue = std::chrono::high_resolution_clock::now();
 }
 
 /**
  * @brief 	Reset attitude estimator to an angle based on an accelerometer measurement
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  */
 void QEKF::Reset(const Eigen::Vector3f accelerometer) {
     Reset();
 
     /* Reset quaternion state into certain angle based on accelerometer measurement */
     // Based on Freescale Application Note: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
     const float mu = 0.0001;  // regularization factor
     float roll =
         atan2f(accelerometer[1], sqrtf(accelerometer[2] * accelerometer[2] + mu * accelerometer[0] * accelerometer[0]));
     float pitch =
         atan2f(-accelerometer[0], sqrtf(accelerometer[1] * accelerometer[1] + accelerometer[2] * accelerometer[2]));
     Quaternion_eul2quat_zyx(0, pitch, roll, &X[0]);
 }
 
 /**
  * @brief 	Reset attitude estimator to an angle based on an accelerometer measurement
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  */
 void QEKF::Reset(const Eigen::Vector3f accelerometer, const float heading) {
     Reset();
 
     /* Reset quaternion state into certain angle based on accelerometer measurement */
     // Based on Freescale Application Note: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
     const float mu = 0.0001;  // regularization factor
     float roll =
         atan2f(accelerometer[1], sqrtf(accelerometer[2] * accelerometer[2] + mu * accelerometer[0] * accelerometer[0]));
     float pitch =
         atan2f(-accelerometer[0], sqrtf(accelerometer[1] * accelerometer[1] + accelerometer[2] * accelerometer[2]));
     Quaternion_eul2quat_zyx(heading, pitch, roll, &X[0]);
 }
 
 /**
  * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
  */
 void QEKF::Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope) {
     Step(accelerometer, gyroscope, _params.estimator.EstimateBias);
 }
 
 /**
  * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
  * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
  */
 void QEKF::Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const bool EstimateBias) {
     float dt;
 
     auto now = std::chrono::high_resolution_clock::now();
     dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - _prevTimerValue).count();
     _prevTimerValue = std::chrono::high_resolution_clock::now();
 
     Step(accelerometer, gyroscope, EstimateBias, dt);
 }
 
 /**
  * @brief 	Estimate attitude quaternion given accelerometer, gyroscope measurements, a heading input/estimate and
  * passed time
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
  * @param	heading			   Input: heading angle in inertial frame [rad]
  * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
  */
 void QEKF::Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const float heading,
                 const bool EstimateBias) {
     float dt;
 
     auto now = std::chrono::high_resolution_clock::now();
     dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - _prevTimerValue).count();
     _prevTimerValue = std::chrono::high_resolution_clock::now();
 
     Step(accelerometer, gyroscope, heading, EstimateBias, dt);
 }
 
 /**
  * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
  * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
  * @param	dt             Input: time passed since last estimate
  */
 void QEKF::Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const bool EstimateBias,
                 const float dt) {
     if (_params.estimator.UseXsensIMU)  // use MTI covariance
         Step(accelerometer, gyroscope, 0, false, _params.estimator.SensorDrivenQEKF, EstimateBias, false,
              _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mti, _params.estimator.cov_gyro_mti,
              _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading,
              _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled,
              _params.estimator.AccelerometerVibration_NormLPFtau,
              _params.estimator.AccelerometerVibration_CovarianceVaryFactor,
              _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
     else
         Step(accelerometer, gyroscope, 0, false, _params.estimator.SensorDrivenQEKF, EstimateBias, false,
              _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mpu, _params.estimator.cov_gyro_mpu,
              _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading,
              _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled,
              _params.estimator.AccelerometerVibration_NormLPFtau,
              _params.estimator.AccelerometerVibration_CovarianceVaryFactor,
              _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
 }
 
 /**
  * @brief 	Estimate attitude quaternion given accelerometer, gyroscope measurements, a heading input/estimate and
  * passed time
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
  * @param	heading			   Input: heading angle in inertial frame [rad]
  * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
  * @param	dt                 Input: time passed since last estimate
  */
 void QEKF::Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const float heading,
                 const bool EstimateBias, const float dt) {
     if (_params.estimator.UseXsensIMU)  // use MTI covariance
         Step(accelerometer, gyroscope, heading, true, _params.estimator.SensorDrivenQEKF, EstimateBias, EstimateBias,
              _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mti, _params.estimator.cov_gyro_mti,
              _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading,
              _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled,
              _params.estimator.AccelerometerVibration_NormLPFtau,
              _params.estimator.AccelerometerVibration_CovarianceVaryFactor,
              _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
     else
         Step(accelerometer, gyroscope, heading, true, _params.estimator.SensorDrivenQEKF, EstimateBias, EstimateBias,
              _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mpu, _params.estimator.cov_gyro_mpu,
              _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading,
              _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled,
              _params.estimator.AccelerometerVibration_NormLPFtau,
              _params.estimator.AccelerometerVibration_CovarianceVaryFactor,
              _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
 }
 
 /**
  * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
  * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
  * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
  * @param	heading            Input: heading angle measurement [rad]
  * @param   UseHeadingForCorrection	 Input: flag to indicate if heading measurement is available
  * @param   SensorDriven       Input: flag to control if QEKF should run in sensor driven mode, disabling smoothing of
  * angular velocity estimate
  * @param   EstimateBias       Input: flag to control if gyroscope x/y axis bias should be estimated
  * @param   EstimateYawBias    Input: flag to control if gyroscope z-axis bias should be estimated
  * @param   CreateQdotFromDifference  Input: flag to control if qdot estimate is generated by differentiating q estimate
  * @param   cov_acc            Input: accelerometer sensor covariance matrix
  * @param   cov_gyro           Input: gyroscope sensor covariance matrix
  * @param   GyroscopeTrustFactor	  Input: tuning factor for accelerometer-gyroscope trust ratio - increase value to
  * trust gyroscope measurements more
  * @param   sigma2_omega       Input: smoothing parameter for angular velocity
  * @param   sigma2_heading     Input: variance on heading input
  * @param   sigma2_bias        Input: bias variance (random walk)
  * @param   AccelerometerVibrationDetectionEnabled  	Input: reduce trust in accelerometer measurements during periods
  * with large vibrations
  * @param   AccelerometerVibrationNormLPFtau  			Input: low-pass filter for vibration detector
  * @param   AccelerometerVibrationCovarianceVaryFactor  Input: exponentially scaled factor to decrease gyroscope
  * covariance (to decrease trust in accelerometer measurement) with during vibration periods
  * @param   AccelerometerCovarianceMaxVaryFactor  		Input: maximum ratio of decrease in gyroscope covariance
  * @param   g                  Input: gravity constant [m/s^2]
  * @param	dt                 Input: time passed since last estimate
  */
 void QEKF::Step(const Eigen::Vector3f accelerometer, const Eigen::Vector3f gyroscope, const float heading,
                 const bool UseHeadingForCorrection, const bool SensorDriven, const bool EstimateBias,
                 const bool EstimateYawBias, const bool CreateQdotFromDifference, const float cov_acc[9],
                 const float cov_gyro[9], const float GyroscopeTrustFactor, const float sigma2_omega,
                 const float sigma2_heading, const float sigma2_bias, const bool AccelerometerVibrationDetectionEnabled,
                 const float AccelerometerVibrationNormLPFtau, const float AccelerometerVibrationCovarianceVaryFactor,
                 const float AccelerometerCovarianceMaxVaryFactor, const float g, const float dt) {
     if (dt == 0) return;  // no time has passed
 
     float X_prev[10];
     memcpy(X_prev, X, sizeof(X_prev));
 
     float P_prev[10 * 10];
     memcpy(P_prev, P, sizeof(P_prev));
 
     float gyroscope_array[3] = {gyroscope[0], gyroscope[1], gyroscope[2]};
     float accelerometer_array[3] = {accelerometer[0], accelerometer[1], accelerometer[2]};
 
     _QEKF(X_prev, P_prev, gyroscope_array, accelerometer_array, heading, UseHeadingForCorrection, dt,
           SensorDriven,  // true == sensor driven Kalman filter
           EstimateBias, EstimateYawBias,
           true,  // true == normalize accelerometer
           cov_gyro, cov_acc, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias,
           AccelerometerVibrationDetectionEnabled, AccelerometerVibrationNormLPFtau,
           AccelerometerVibrationCovarianceVaryFactor, AccelerometerCovarianceMaxVaryFactor, g, X, P);
 
     Math_SymmetrizeSquareMatrix(P, sizeof(X) / sizeof(float));
 
     if (CreateQdotFromDifference) {
         X[4] = (X[0] - X_prev[0]) / dt;  // dq[0]
         X[5] = (X[1] - X_prev[1]) / dt;  // dq[1]
         X[6] = (X[2] - X_prev[2]) / dt;  // dq[2]
         X[7] = (X[3] - X_prev[3]) / dt;  // dq[3]
     }
 }
 
 /**
  * @brief 	Get estimated attitude quaternion
  * @param	q[4]		Output: estimated attitude quaternion
  */
 void QEKF::GetQuaternion(Eigen::Quaternionf& q) {
     q.w() = X[0];
     q.x() = X[1];
     q.y() = X[2];
     q.z() = X[3];
 }
 
 /**
  * @brief 	Get estimated attitude quaternion derivative
  * @param	dq[4]		Output: estimated attitude quaternion derivative
  */
 void QEKF::GetQuaternionDerivative(Eigen::Vector4f& dq_) {
     /* Body angular velocity */
     /* dq = 1/2 * Phi(q) * [0;omega]; */
     float dq[4];
     float omega_q[4] = {0, X[4], X[5], X[6]};
     Quaternion_Phi(&X[0], omega_q, dq);  // Phi(q) * [0;omega]
     math_utils::scale_f32(dq, 0.5f, dq, 4);
     // arm_scale_f32(dq, 0.5f, dq, 4);
     dq_[0] = dq[0];
     dq_[1] = dq[1];
     dq_[2] = dq[2];
     dq_[3] = dq[3];
 
     /*dq[0] = X[4];
     dq[1] = X[5];
     dq[2] = X[6];
     dq[3] = X[7];*/
 }
 
 /**
  * @brief 	Get estimated gyroscope bias
  * @param	bias[3]		Output: estimated gyroscope bias
  */
 void QEKF::GetGyroBias(float bias[3]) {
     bias[0] = X[7];
     bias[1] = X[8];
     bias[2] = X[9];
 }
 
 /**
  * @brief 	Get covariance matrix of estimated quaternion
  * @param	Cov_q[4*4]		Output: quaternion estimate covariance
  */
 void QEKF::GetQuaternionCovariance(Eigen::Matrix<float, 4, 4>& Cov_q) {
     for (int m = 0; m < 4; m++) {
         for (int n = 0; n < 4; n++) {
             Cov_q(m, n) = P[10 * m + n];
         }
     }
 }
 
 /**
  * @brief 	Get covariance matrix of estimated quaternion derivative
  * @param	Cov_dq[4*4]		Output: quaternion derivative estimate covariance
  */
 void QEKF::GetQuaternionDerivativeCovariance(float Cov_dq[4 * 4]) {  //!!! may have problem (related to Eigen::ColMajor)
     /*for (int m = 0; m < 4; m++) {
       for (int n = 0; n < 4; n++) {
         Cov_dq[4*m + n] = P[10*m + n + (10*4 + 4)];
       }
     }*/
 
     // OBS. The covariance of the quaternion derivative estimate is not stored in the estimator covariance, since it is
     // not part of the state vector Hence we need to transform the covariance of the angular velocity estimate into a
     // covariance of the quaternion derivative estimate
 
     /* Cov_dq = (1/2 * Phi(q) * vec) * Cov_omega * (1/2 * Phi(q) * vec)' */
     /* Cov_dq = T(q) * Cov_omega * T(q)' */
     float Cov_omega[3 * 3];
     Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> Cov_omega_{Cov_omega};
     GetAngularVelocityCovariance(Cov_omega);
     // arm_matrix_instance_f32 Cov_omega_;
     // arm_mat_init_f32(&Cov_omega_, 3, 3, Cov_omega);
 
     // Compute transformation matrix, T(q)
     float T_q[4 * 3];
     Eigen::Map<Eigen::Matrix<float, 4, 3, Eigen::RowMajor>> T_q_{T_q};
     // arm_matrix_instance_f32 T_q_;
     // arm_mat_init_f32(&T_q_, 4, 3, T_q);
     Quaternion_mat_PhiVec(&X[0], T_q);
     math_utils::scale_f32(T_q, 0.5f, T_q, 4 * 3);
     // arm_scale_f32(T_q, 0.5f, T_q, 4*3);
 
     // Compute transpose, T(q)'
     float T_q_T[3 * 4];
     Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>> T_q_T_{T_q_T};
     T_q_T_ = T_q_.transpose();
     // arm_matrix_instance_f32 T_q_T_;
     // arm_mat_init_f32(&T_q_T_, 3, 4, T_q_T);
     // arm_mat_trans_(f32&T_q_, &T_q_T_);
 
     // Compute right part of transformation   -->   tmp = Cov_omega * T(q)'
     float tmp[3 * 4];
     Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>> tmp_{tmp};
     tmp_.noalias() = Cov_omega_ * T_q_T_;
     // arm_matrix_instance_f32 tmp_;
     // arm_mat_init_f32(&tmp_, 3, 4, tmp);
     // arm_mat_mult_f32(&Cov_omega_, &T_q_T_, &tmp_);
 
     // Compute output   -->  Cov_dq = T(q) * tmp
     Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> Cov_dq_{Cov_dq};
     Cov_dq_ = T_q_ * tmp_;
     // arm_matrix_instance_f32 Cov_dq_;
     // arm_mat_init_f32(&Cov_dq_, 4, 4, Cov_dq);
     // arm_mat_mult_f32(&T_q_, &tmp_, &Cov_dq_);
 
     Math_SymmetrizeSquareMatrix(Cov_dq, 4);
 }
 
 /**
  * @brief 	Get covariance matrix of estimated angular velocity
  * @param	Cov_omega[3*3]		Output: angular velocity estimate covariance
  */
 void QEKF::GetAngularVelocityCovariance(float Cov_omega[3 * 3]) {
     for (int m = 0; m < 3; m++) {
         for (int n = 0; n < 3; n++) {
             Cov_omega[3 * m + n] = P[10 * m + n + (10 * 4 + 4)];
         }
     }
 }
 
 /**
  * @brief 	Get covariance matrix of estimated gyroscope bias
  * @param	Cov_bias[3*3]		Output: gyroscope bias estimate covariance
  */
 void QEKF::GetBiasCovariance(float Cov_bias[3 * 3]) {
     for (int m = 0; m < 3; m++) {
         for (int n = 0; n < 3; n++) {
             Cov_bias[3 * m + n] = P[10 * m + n + (10 * 7 + 7)];
         }
     }
 }