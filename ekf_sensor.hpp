#ifndef EKF_SENSER_HPP
#define EKF_SENSER_HPP

#include <random>
#include <unistd.h>
#include <Eigen/Dense>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "pico/stdlib.h"
#include <string.h>
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "lsm9ds1_reg.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

#define PIN_CSAG 13
#define PIN_CSM  9
#define PIN_MISO 8
#define PIN_SCK  10
#define PIN_MOSI 11
#define GRAV (9.80665)
#define PI (3.14159)

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix4f;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

//extern  int16_t data_raw_acceleration[3];
//extern  int16_t data_raw_angular_rate[3];
//extern  int16_t data_raw_magnetic_field[3];
extern  float acceleration_mg[3];
extern  float angular_rate_mdps[3];
extern  float magnetic_field_mgauss[3];
//extern sensbus_t Ins_bus;
//extern sensbus_t Mag_bus;
//extern stmdev_ctx_t Imu_h;
//extern stmdev_ctx_t Mag_h;
extern float MN,ME,MD;

void imu_mag_data_read(void);
void imu_mag_init(void);
float CalcPsi(Matrix<float, 7, 1>x);
float CalcTheta(Matrix<float, 7, 1>x);
float CalcPhi(Matrix<float, 7, 1>x);
uint8_t ekf( Matrix<float, 7, 1> &xe,
             Matrix<float, 7, 1> &xp,
             Matrix<float, 7, 7> &P,
             Matrix<float, 6, 1> z,
             Matrix<float, 3, 1> omega,
             Matrix<float, 6, 6> Q, 
             Matrix<float, 6, 6> R, 
             Matrix<float, 7, 6> G,
             Matrix<float, 3, 1> beta,
             float dt);
#endif
