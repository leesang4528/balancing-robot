#define F_CPU 8000000UL

#include "mpu6050.h"
#include "i2c.h"

#include <util/delay.h>
#include <math.h>

// ---------------- 공통 상수 ----------------
#define RAD2DEG   (180.0 / 3.14159265358979323846)

// main에서 쓰던 값 그대로 유지 (링커 충돌 피하려고 여기서 정의)
const double ALPHA = 0.99;//96//98
const double ALPHA0 = 0.99;
const double DT    = 0.008;

// ---------------- MPU6050 ----------------
#define MPU6050_ADDR      0x68
#define MPU6050_ADDR_W    (MPU6050_ADDR << 1)
#define MPU6050_ADDR_R    ((MPU6050_ADDR << 1) | 1)

#define MPU6050_RA_PWR_MGMT_1     0x6B
#define MPU6050_RA_SMPLRT_DIV     0x19
#define MPU6050_RA_CONFIG         0x1A
#define MPU6050_RA_GYRO_CONFIG    0x1B
#define MPU6050_RA_ACCEL_CONFIG   0x1C
#define MPU6050_RA_INT_ENABLE     0x38
#define MPU6050_RA_ACCEL_XOUT_H   0x3B
#define MPU6050_RA_WHO_AM_I       0x75

// globals
double gyroX_offset = 0.0;
double gyroY_offset = 0.0;
double gyroY0_offset = 0.0;


int16_t ax, ay, az, gx, gy, gz;

double accelAngleX = 0.0;
double accelAngleY = 0.0;
double accelAngleY0 = 0.0;


double angleX = 0.0;
double angleY = 0.0;
double angleY2 = 0.0;  /* alpha = 0.96 */
double angleY95 = 0.0;
double angleY97 = 0.0;
double angleY98 = 0.0;

double angleY0 = 0.0;

double gx_dps = 0.0;
// mpu6050.c 상단 globals에 추가
double gy_dps = 0.0;
double gx_f   = 0.0;
double gy_f = 0.0;
double gy0_dps = 0.0;
double gy0_f = 0.0;

double gy_f2 = 0.0;

//  1차 저역통과 필터 계수
#define GX_BETA  0.85 //노이즈 감소, 반응 약간 느려짐
#define GY_BETA  0.80 //고정

static void mpu6050_write_reg(uint8_t reg, uint8_t data)
{
	i2c_start(MPU6050_ADDR_W);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}

static void mpu6050_read_bytes(uint8_t start_reg, uint8_t* buf, uint8_t len)
{
	i2c_start(MPU6050_ADDR_W);
	i2c_write(start_reg);
	i2c_restart(MPU6050_ADDR_R);

	for (uint8_t i = 0; i < len; i++)
	{
		if (i == len - 1) buf[i] = i2c_read_nack();
		else              buf[i] = i2c_read_ack();
	}
	i2c_stop();
}

void MPU6050_init(void)
{
	i2c_init(400000UL);
	_delay_ms(100);

	mpu6050_write_reg(MPU6050_RA_PWR_MGMT_1, 0x01);
	_delay_ms(10);

	mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV,  0x07);
	mpu6050_write_reg(MPU6050_RA_CONFIG,      0x03); // DLPF 42Hz
	mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, 0x08);
	mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG,0x08);
	mpu6050_write_reg(MPU6050_RA_INT_ENABLE,  0x00);
}

uint8_t MPU6050_test(void)
{
	uint8_t who = 0;
	mpu6050_read_bytes(MPU6050_RA_WHO_AM_I, &who, 1);
	return (who == 0x68);
}

void MPU6050_read_all(int16_t* pax, int16_t* pay, int16_t* paz,
int16_t* pgx, int16_t* pgy, int16_t* pgz)
{
	uint8_t buf[14];
	mpu6050_read_bytes(MPU6050_RA_ACCEL_XOUT_H, buf, 14);

	*pax = (int16_t)((buf[0] << 8) | buf[1]);
	*pay = (int16_t)((buf[2] << 8) | buf[3]);
	*paz = (int16_t)((buf[4] << 8) | buf[5]);

	*pgx = (int16_t)((buf[8]  << 8) | buf[9]);
	*pgy = (int16_t)((buf[10] << 8) | buf[11]);
	*pgz = (int16_t)((buf[12] << 8) | buf[13]);
}

void calc_accel_angle(void)
{
	double axd = (double)ax;
	double ayd = (double)ay;
	double azd = (double)az;
	
	accelAngleX = atan2((double)ayd,(double)azd) * RAD2DEG;  // az 기준
	accelAngleY0 = atan2(-(double)axd, (double)azd) * RAD2DEG;
	accelAngleY = atan2(-(double)axd,sqrt((double)ayd*ayd+(double)azd*azd)) * RAD2DEG;
	
}

void gyro_calibrate(void)
{
	long sumX = 0, sumY = 0;
	int16_t cax, cay, caz, cgx, cgy, cgz;

	for (uint16_t i = 0; i < 500; i++)
	{
		MPU6050_read_all(&cax, &cay, &caz, &cgx, &cgy, &cgz);
		sumX += cgx;
		sumY += cgy;
		_delay_ms(10);
	}
	gyroX_offset = (double)sumX / 500.0;
	gyroY_offset = (double)sumY / 500.0;
	gyroY0_offset = (double)sumY / 500.0;
}

void complementary_update(void)
{
	gx_dps = ((double)gx - gyroX_offset) / 65.5;
	gy_dps = ((double)gy - gyroY_offset) / 65.5;
	gy0_dps = ((double)gy - gyroY0_offset) / 65.5;


	// 1단 LPF
	gx_f = GX_BETA * gx_f + (1.0 - GX_BETA) * gx_dps;
	gy_f = GY_BETA * gy_f + (1.0 - GY_BETA) * gy_dps;
	gy0_f = GY_BETA * gy0_f + (1.0 - GY_BETA) * gy0_dps;

	// 2단 LPF (D항용)
	gy_f2 = 0.60 * gy_f2 + 0.40 * gy_f;

	// 데드존
	if (fabs(gy_f2) < 0.5) gy_f2 = 0.0;

	// 클램핑 (계산 후에 한 번씩만)
	const double GY_LIM  = 50.0;
	const double GYF_LIM = 350.0;
	if (gy_f2 >  GY_LIM)  gy_f2 =  GY_LIM;
	if (gy_f2 < -GY_LIM)  gy_f2 = -GY_LIM;
	if (gy_f  >  GYF_LIM) gy_f  =  GYF_LIM;
	if (gy_f  < -GYF_LIM) gy_f  = -GYF_LIM;
	if (gy0_f  >  GYF_LIM) gy0_f  =  GYF_LIM;
	if (gy0_f  < -GYF_LIM) gy0_f  = -GYF_LIM;


	angleX = ALPHA * (angleX + gx_dps * DT) + (1.0 - ALPHA) * accelAngleX;
	angleY = ALPHA * (angleY + gy_dps   * DT) + (1.0 - ALPHA) * accelAngleY;
	
	angleY95 = 0.95 * (angleY95 + gy_f * DT) + 0.05 * accelAngleY;
	angleY2  = 0.96 * (angleY2  + gy_f * DT) + 0.04 * accelAngleY;
	angleY97 = 0.97 * (angleY97 + gy_f * DT) + 0.03 * accelAngleY;
	angleY98 = 0.98 * (angleY98 + gy_f * DT) + 0.02 * accelAngleY;
	angleY0 = ALPHA * (angleY0 + gy0_f   * DT) + (1.0 - ALPHA) * accelAngleY0;
}
/*
void complementary_update(void)
{
	gx_dps = ((double)gx - gyroX_offset) / 65.5;
	gy_dps = ((double)gy - gyroY_offset) / 65.5;

	gx_f = GX_BETA * gx_f + (1.0 - GX_BETA) * gx_dps;
	 // Y축 필터 추가
	 gy_f = GY_BETA * gy_f + (1.0 - GY_BETA) * gy_dps;
	   gy_f2 = GX_BETA * gy_f2 + (1.0 - GX_BETA) * gy_f;

	   if (fabs(gy_f2) < 2.0) gy_f2 = 0.0;
	   const double GXF_LIM = 350.0; // 수정
	   if (gy_f2 >  GXF_LIM) gy_f2 =  GXF_LIM;
	   if (gy_f2 < -GXF_LIM) gy_f2 = -GXF_LIM;


	angleX = ALPHA * (angleX + gx_dps * DT) + (1.0 - ALPHA) * accelAngleX;
	angleY = ALPHA * (angleY + gy_f * DT) + (1.0 - ALPHA) * accelAngleY;
}*/


