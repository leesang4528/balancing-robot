#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

// raw
extern int16_t ax, ay, az, gx, gy, gz;

// angles
extern double accelAngleX, accelAngleY, accelAngleY0;
extern double angleX, angleY, angleY0,angleY2;
extern double angleY95, angleY97, angleY98;


// gyro filter
extern double gx_dps;
extern double gx_f;
extern double gy_dps;
extern double gy0_dps;
extern double gy_f;
extern double gy0_f;
extern double gy_f2;

// calibration
extern double gyroX_offset, gyroY_offset,gyroY0_offset;

// config
extern const double DT;
extern const double ALPHA;
extern const double ALPHA0;

// APIs
void MPU6050_init(void);
uint8_t MPU6050_test(void);

void MPU6050_read_all(int16_t* pax, int16_t* pay, int16_t* paz,
int16_t* pgx, int16_t* pgy, int16_t* pgz);

void calc_accel_angle(void);
void gyro_calibrate(void);
void complementary_update(void);

#endif