/*
 * imu_roll_pitch.c
 *
 * Created: 2026-03-10 오후 3:33:48
 * Author : 오소현
 */ 
/*
 * 1120_imu_test_final_GY521.c
 *
 * GY-521(MPU6050) + ATmega64A
 * - I2C 100kHz
 * - Gyro offset calibration
 * - Accel angle average init
 * - Complementary filter (angle-based, recommended)
 * - Runtime gyro bias update (slow drift compensation)
 */

#define F_CPU 8000000UL
#define BAUD   38400
#define MYUBRR (F_CPU/(16UL*BAUD)-1)


#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
#include <stdio.h>
#include "USART_ch0.h"

// ------------------------------------------------------
// 상수/파라미터
// ------------------------------------------------------
#define I2C_OK   0
#define I2C_ERR  1

#define RAD2DEG (180.0 / 3.14159265358979323846)

// 상보필터 비율 (GY-521에서 보통 0.94~0.97)
#define ALPHA   0.96

// 루프 주기(초)
// _delay_ms(10) + 연산 오버헤드 감안해서 0.012~0.015로 튜닝 가능
#define DT      0.011

// ------------------------------------------------------
// I2C(TWI) 기본
// ------------------------------------------------------
void i2c_init(uint32_t scl_hz)
{
    // prescaler = 1
    TWSR = 0x00;
    uint32_t twbr_val = ((F_CPU / scl_hz) - 16) / 2;
    TWBR = (uint8_t)twbr_val;
}

uint8_t i2c_start(uint8_t address)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    TWDR = address;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    uint8_t status = TWSR & 0xF8;
    if (status != TW_MT_SLA_ACK && status != TW_MR_SLA_ACK)
        return I2C_ERR;

    return I2C_OK;
}

uint8_t i2c_restart(uint8_t address)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    TWDR = address;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    uint8_t status = TWSR & 0xF8;
    if (status != TW_MT_SLA_ACK && status != TW_MR_SLA_ACK)
        return I2C_ERR;

    return I2C_OK;
}

void i2c_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    uint8_t status = TWSR & 0xF8;
    if (status != TW_MT_DATA_ACK)
        return I2C_ERR;

    return I2C_OK;
}

uint8_t i2c_read_ack(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t i2c_read_nack(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

// ------------------------------------------------------
// MPU6050 (GY-521)
// ------------------------------------------------------
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

int16_t ax, ay, az, gx, gy, gz;

double gyroX_offset = 0.0;
double gyroY_offset = 0.0;

double accelAngleX = 0.0;   // accel 기반 roll
double accelAngleY = 0.0;   // accel 기반 pitch

double angleX = 0.0;        // 최종 roll
double angleY = 0.0;        // 최종 pitch

void mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    i2c_start(MPU6050_ADDR_W);
    i2c_write(reg);
    i2c_write(data);
    i2c_stop();
}

void mpu6050_read_bytes(uint8_t start_reg, uint8_t* buf, uint8_t len)
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
    // 100kHz로 안정화
    i2c_init(100000UL);
    _delay_ms(100);

    mpu6050_write_reg(MPU6050_RA_PWR_MGMT_1, 0x00);
    _delay_ms(10);

    mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV, 0x07);
    mpu6050_write_reg(MPU6050_RA_CONFIG,     0x03); // DLPF 42Hz
    mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG,0x08); // ±500 dps
    mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG,0x08);// ±4g
    mpu6050_write_reg(MPU6050_RA_INT_ENABLE, 0x00);
}

uint8_t MPU6050_test(void)
{
    uint8_t who = 0;
    mpu6050_read_bytes(MPU6050_RA_WHO_AM_I, &who, 1);
    return (who == 0x68);
}

void MPU6050_read_all(int16_t* ax, int16_t* ay, int16_t* az,
                      int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buf[14];
    mpu6050_read_bytes(MPU6050_RA_ACCEL_XOUT_H, buf, 14);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);

    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// ------------------------------------------------------
// 각도 계산 (GY-521 표준 Roll/Pitch)
// ------------------------------------------------------
void calc_accel_angle(void)
{
	// Roll (X축: 좌/우 기울기)
	accelAngleX = atan2((double)ay, (double)az) * RAD2DEG;
	// Pitch (Y축: 앞/뒤 기울기)
	accelAngleY = atan2(-(double)ax, sqrt((double)ay*ay + (double)az*az)) * RAD2DEG;
}
// ------------------------------------------------------
// 자이로 캘리브레이션 (정지 상태에서만!)
// ------------------------------------------------------
void gyro_calibrate(void)
{
    long sumX = 0, sumY = 0;
    int16_t cax, cay, caz, cgx, cgy, cgz;

    for (uint16_t i = 0; i < 200; i++)
    {
        MPU6050_read_all(&cax, &cay, &caz, &cgx, &cgy, &cgz);
        sumX += cgx;
        sumY += cgy;
        _delay_ms(5);
    }

    gyroX_offset = (double)sumX / 200.0;
    gyroY_offset = (double)sumY / 200.0;
}

// ------------------------------------------------------
// 상보필터 (angle 기반 정석 형태)
// ------------------------------------------------------
void complementary_update(void)
{
    // 자이로 raw -> dps 변환
    double gx_dps = ((double)gx - gyroX_offset) / 65.5;
    double gy_dps = ((double)gy - gyroY_offset) / 65.5;

    // 이전 최종각에 자이로 적분을 먼저 더하고,
    // 가속도 각도로 천천히 보정
    angleX = ALPHA * (angleX + gx_dps * DT) + (1.0 - ALPHA) * accelAngleX;
    angleY = ALPHA * (angleY + gy_dps * DT) + (1.0 - ALPHA) * accelAngleY;
}

// ------------------------------------------------------
// 런타임 드리프트 미세보정 (천천히 따라가게)
// ------------------------------------------------------
void gyro_bias_update(void)
{
    double errX = angleX - accelAngleX;
    double errY = angleY - accelAngleY;

    // 너무 크면 불안정해질 수 있음 (0.001~0.02에서 튜닝)
    gyroX_offset += errX * 0.01;
    gyroY_offset += errY * 0.01;
}

// ------------------------------------------------------
// main
// ------------------------------------------------------
int main(void)
{
    USART0_Init(MYUBRR);
    USART0_Transmit_String("UART 38400 start\n");
    _delay_ms(100);

    MPU6050_init();

    while (!MPU6050_test())
    {
        USART0_Transmit_String("MPU6050 not detected\n");
        _delay_ms(300);
    }

    // 1) 자이로 오프셋 캘리브레이션
    gyro_calibrate();

    // 2) 초기 자세(기준 자세)를 accel로 평균내서 angle 초기값 안정화
    double sumX = 0, sumY = 0;
    for (int i = 0; i < 100; i++)
    {
        MPU6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);
        calc_accel_angle();
        sumX += accelAngleX;
        sumY += accelAngleY;
        _delay_ms(5);
    }
    angleX = sumX / 100.0;
    angleY = sumY / 100.0;

    char msg[64];
 
    while (1)
    {
        MPU6050_read_all(&ax, &ay, &az, &gx, &gy, &gz);

        calc_accel_angle();
        complementary_update();
        gyro_bias_update();

        // 최종 각도 출력
        sprintf(msg, "ax:%d ay:%d az:%d | AX:%.1f AY:%.1f\r\n",ax, ay, az, angleX, angleY);
        USART0_Transmit_String(msg);
        _delay_ms(10);
    }
}

