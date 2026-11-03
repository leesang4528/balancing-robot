#define F_CPU 8000000UL

#define BAUD   38400UL
#define MYUBRR (F_CPU / (16UL * BAUD) - 1UL)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "USART_ch_inter.h"
#include "i2c.h"
#include "mpu6050.h"

/* =========================================================
 * 모터 핀
 * ========================================================= */

#define STEP_PORT  PORTB
#define STEP_DDR   DDRB
#define STEP1_PIN  PB5
#define STEP2_PIN  PB6

#define DIR_PORT  PORTF
#define DIR_DDR   DDRF
#define DIR1_PIN  PF0
#define DIR2_PIN  PF2

#define EN_PORT  PORTF
#define EN_DDR   DDRF
#define EN1_PIN  PF1
#define EN2_PIN  PF3

/* =========================================================
 * DRV8825 마이크로스텝 핀
 *
 * PA0 → 두 DRV8825 MS1
 * PA1 → 두 DRV8825 MS2
 * PA2 → 두 DRV8825 MS3
 * ========================================================= */

#define MS_PORT   PORTA
#define MS_DDR    DDRA

#define MS1_PIN   PA0
#define MS2_PIN   PA1
#define MS3_PIN   PA2

/* =========================================================
 * 자세 PD
 * ========================================================= */

#define STEP_MIN_HZ  120U
#define STEP_MAX_HZ  3000U

#define CMD_MAX       275.0
#define CMD_DEADZONE  0.5
#define FALL_ANGLE    30.0

#define KP  5.0
#define KD  1.3

/* =========================================================
 * 제어주기
 * ========================================================= */

#define CONTROL_DT         0.008
#define CONTROL_PERIOD_MS  8UL

/* =========================================================
 * 바퀴 및 STEP 설정
 * ========================================================= */

#define WHEEL_RADIUS_MM    40.0
#define FULL_STEP_PER_REV  200.0
#define MICROSTEP          32.0

#define STEP_PER_REV \
    (FULL_STEP_PER_REV * MICROSTEP)

#define MM_PER_STEP \
    ((2.0 * 3.14159265358979323846 * WHEEL_RADIUS_MM) \
    / STEP_PER_REV)

/* =========================================================
 * 위치 PI
 * ========================================================= */

#define POSITION_KP  0.00
#define POSITION_KI  0.000

#define POSITION_I_LIMIT  300.0
#define POSITION_ANGLE_LIMIT  2.0

#define MOVE_START_COUNT  300UL
#define TARGET_POSITION_MM  100.0

/* =========================================================
 * 전역 변수
 * ========================================================= */

volatile uint32_t system_ms = 0;

volatile uint16_t last_hz = 0;

volatile int32_t step_count = 0;
volatile int8_t step_direction = 0;
volatile uint8_t step_output_active = 0;

/* =========================================================
 * 범위 제한
 * ========================================================= */

static double clamp_double(
    double value,
    double minimum,
    double maximum
)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

/* =========================================================
 * 마이크로스텝 설정
 * ========================================================= */

static void microstep_init(void)
{
    MS_DDR |=
        (1 << MS1_PIN) |
        (1 << MS2_PIN) |
        (1 << MS3_PIN);
}

/* Full Step
 * MS1 = 0
 * MS2 = 0
 * MS3 = 0
 */
static void set_microstep_full(void)
{
    MS_PORT &=
        ~(
            (1 << MS1_PIN) |
            (1 << MS2_PIN) |
            (1 << MS3_PIN)
        );
}

/* 1/8 Step
 * MS1 = 1
 * MS2 = 1
 * MS3 = 0
 */
static void set_microstep_8(void)
{
    MS_PORT |=
        (1 << MS1_PIN) |
        (1 << MS2_PIN);

    MS_PORT &=
        ~(1 << MS3_PIN);
}

/* 1/16 Step
 * MS1 = 0
 * MS2 = 0
 * MS3 = 1
 */
static void set_microstep_16(void)
{
    MS_PORT &=
        ~(
            (1 << MS1_PIN) |
            (1 << MS2_PIN)
        );

    MS_PORT |=
        (1 << MS3_PIN);
}

/* 1/32 Step
 * MS1 = 1
 * MS2 = 0
 * MS3 = 1
 */
static void set_microstep_32(void)
{
    MS_PORT |=
        (1 << MS1_PIN) |
        (1 << MS3_PIN);

    MS_PORT &=
        ~(1 << MS2_PIN);
}

/* =========================================================
 * Timer0: 1ms 시스템 tick
 * ========================================================= */

ISR(TIMER0_COMP_vect)
{
    system_ms++;
}

static void control_timer_init(void)
{
    TCCR0 =
        (1 << WGM01) |
        (1 << CS02);

    OCR0  = 124;
    TCNT0 = 0;

    TIMSK |= (1 << OCIE0);
}

static uint32_t millis_snapshot(void)
{
    uint8_t saved_sreg;
    uint32_t value;

    saved_sreg = SREG;

    cli();
    value = system_ms;
    SREG = saved_sreg;

    return value;
}

/* =========================================================
 * STEP 위치 안전 복사
 * ========================================================= */

static int32_t step_count_snapshot(void)
{
    uint8_t saved_sreg;
    int32_t value;

    saved_sreg = SREG;

    cli();
    value = step_count;
    SREG = saved_sreg;

    return value;
}

static void step_count_reset(void)
{
    uint8_t saved_sreg;

    saved_sreg = SREG;

    cli();
    step_count = 0;
    SREG = saved_sreg;
}

/* =========================================================
 * Timer1 STEP 계수
 * ========================================================= */

ISR(TIMER1_OVF_vect)
{
    if (step_output_active)
    {
        step_count += step_direction;
    }
}

/* =========================================================
 * Timer1 모터 초기화
 * ========================================================= */

static void motor_timer_init(void)
{
    uint16_t top;

    STEP_DDR |=
        (1 << STEP1_PIN) |
        (1 << STEP2_PIN);

    /*
     * Fast PWM
     * TOP = ICR1
     */
    TCCR1A =
        (1 << COM1A1) |
        (1 << COM1B1) |
        (1 << WGM11);

    /*
     * 분주비 8
     */
    TCCR1B =
        (1 << WGM13) |
        (1 << WGM12) |
        (1 << CS11);

    top =
        (uint16_t)(
            (F_CPU / 8UL) /
            1000UL -
            1UL
        );

    ICR1  = top;
    OCR1A = (top + 1U) / 2U;
    OCR1B = (top + 1U) / 2U;

    TIMSK |= (1 << TOIE1);
}

/* =========================================================
 * 모터 GPIO 초기화
 * ========================================================= */

static void motor_io_init(void)
{
    DIR_DDR |=
        (1 << DIR1_PIN) |
        (1 << DIR2_PIN);

    DIR_PORT &= ~(1 << DIR1_PIN);
    DIR_PORT &= ~(1 << DIR2_PIN);

    EN_DDR |=
        (1 << EN1_PIN) |
        (1 << EN2_PIN);

    /*
     * DRV8825 nEN
     * LOW = 활성화
     */
    EN_PORT &=
        ~(
            (1 << EN1_PIN) |
            (1 << EN2_PIN)
        );
}

/* =========================================================
 * STEP 출력 정지
 * ========================================================= */

static void motor_stop_step_output(void)
{
    step_output_active = 0;

    TCCR1A &=
        ~(
            (1 << COM1A1) |
            (1 << COM1B1)
        );

    STEP_PORT &=
        ~(
            (1 << STEP1_PIN) |
            (1 << STEP2_PIN)
        );

    /*
     * 드라이버 활성 상태 유지 → 홀딩토크 유지
     */
    EN_PORT &=
        ~(
            (1 << EN1_PIN) |
            (1 << EN2_PIN)
        );

    last_hz = 0;
}

/* =========================================================
 * STEP 주파수 설정
 * ========================================================= */

static void motor_set_step_hz(uint16_t hz)
{
    uint32_t top32;
    uint16_t top;

    if (hz < STEP_MIN_HZ)
    {
        motor_stop_step_output();
        return;
    }

    if (hz > STEP_MAX_HZ)
    {
        hz = STEP_MAX_HZ;
    }

    top32 =
        (F_CPU / 8UL) /
        (uint32_t)hz -
        1UL;

    if (top32 > 65534UL)
    {
        top32 = 65534UL;
    }

    top = (uint16_t)top32;

    if (TCNT1 >= top)
    {
        TCNT1 = 0;
    }

    ICR1  = top;
    OCR1A = (top + 1U) / 2U;
    OCR1B = (top + 1U) / 2U;

    EN_PORT &=
        ~(
            (1 << EN1_PIN) |
            (1 << EN2_PIN)
        );

    TCCR1A |=
        (1 << COM1A1) |
        (1 << COM1B1);

    step_output_active = 1;
    last_hz = hz;
}

/* =========================================================
 * 자세 PD 출력을 모터 명령으로 변환
 *
 * 기존 코드에서 모터 방향만 반대로 변경
 * ========================================================= */

static void motor_set_speed(double cmd)
{
    uint16_t target_hz;

    if (fabs(cmd) < CMD_DEADZONE)
    {
        motor_set_step_hz(0);
        return;
    }

    cmd =
        clamp_double(
            cmd,
            -CMD_MAX,
            CMD_MAX
        );

    /*
     * 방향 반전
     */
    if (cmd > 0.0)
    {
        DIR_PORT |=  (1 << DIR1_PIN);
        DIR_PORT &= ~(1 << DIR2_PIN);

        step_direction = 1;
    }
    else
    {
        DIR_PORT &= ~(1 << DIR1_PIN);
        DIR_PORT |=  (1 << DIR2_PIN);

        step_direction = -1;
    }

    target_hz =
        (uint16_t)(
            STEP_MIN_HZ +
            fabs(cmd) / CMD_MAX *
            (STEP_MAX_HZ - STEP_MIN_HZ)
        );

    motor_set_step_hz(target_hz);
}

/* =========================================================
 * main
 * ========================================================= */

int main(void)
{
    char msg[96];

    double sumX;
    double sumY;
    double sumY0;

    double targetAngleX;
    double targetAngleY;
    double targetAngleY0;

    double angleY_zero;
    double gyro_angle_y;

    double target_position_mm;
    double current_position_mm;
    double position_error_mm;
    double position_integral;
    double position_angle_offset;

    double angle_target;
    double error;
    double cmd;

    int32_t current_step;

    uint32_t control_count;
    uint32_t next_control_ms;
    uint32_t now_ms;

    uint8_t move_started;
    uint8_t print_div;

    int i;

    /* -----------------------------------------------------
     * 초기화
     * ----------------------------------------------------- */

    USART0_Init((INT16)MYUBRR);
    MPU6050_init();

    while (!MPU6050_test())
    {
        _delay_ms(500);
    }

    /* -----------------------------------------------------
     * 마이크로스텝 설정
     * ----------------------------------------------------- */

    microstep_init();

    /*
     * 현재 1/16 마이크로스텝
     *
     * PA0 = LOW
     * PA1 = LOW
     * PA2 = HIGH
     */
    set_microstep_32();

    /* -----------------------------------------------------
     * 모터 초기화
     * ----------------------------------------------------- */

    motor_io_init();
    motor_timer_init();

    /*
     * 여기에는 motor_stop_step_output()을 넣지 않음.
     * 따라서 기존처럼 캘리브레이션 동안의 모터 동작은 유지.
     */

    gyro_calibrate();

    MPU6050_read_all(
        &ax, &ay, &az,
        &gx, &gy, &gz
    );

    calc_accel_angle();

    angleX  = accelAngleX;
    angleY  = accelAngleY;
    angleY2 = accelAngleY;
    angleY0 = accelAngleY0;

    /* -----------------------------------------------------
     * 초기 수직 기준각 측정
     * ----------------------------------------------------- */

    sumX  = 0.0;
    sumY  = 0.0;
    sumY0 = 0.0;

    for (i = 0; i < 200; i++)
    {
        MPU6050_read_all(
            &ax, &ay, &az,
            &gx, &gy, &gz
        );

        calc_accel_angle();

        sumX  += accelAngleX;
        sumY  += accelAngleY;
        sumY0 += accelAngleY0;

        _delay_ms(10);
    }

    targetAngleX  = sumX  / 200.0;
    targetAngleY  = sumY  / 200.0;
    targetAngleY0 = sumY0 / 200.0;

    angleX  = targetAngleX;
    angleY  = targetAngleY;
    angleY2 = targetAngleY;
    angleY0 = targetAngleY0;

    angleY_zero = targetAngleY;
    gyro_angle_y = angleY_zero;

    gy_f  = 0.0;
    gy0_f = 0.0;
    gy_f2 = 0.0;

    step_count_reset();

    target_position_mm = 0.0;
    current_position_mm = 0.0;
    position_error_mm = 0.0;
    position_integral = 0.0;
    position_angle_offset = 0.0;

    control_count = 0;
    move_started = 0;
    print_div = 0;

    /* -----------------------------------------------------
     * 타이머 및 인터럽트 시작
     * ----------------------------------------------------- */

    control_timer_init();
    sei();

    next_control_ms =
        millis_snapshot() +
        CONTROL_PERIOD_MS;

    /* =====================================================
     * 8ms 제어 루프
     * ===================================================== */

    while (1)
    {
        while (
            (int32_t)(
                millis_snapshot() -
                next_control_ms
            ) < 0
        )
        {
            ;
        }

        next_control_ms +=
            CONTROL_PERIOD_MS;

        now_ms = millis_snapshot();

        /*
         * 제어가 크게 지연된 경우
         * 새로운 8ms 기준 설정
         */
        if (
            (int32_t)(
                now_ms -
                next_control_ms
            ) >= 0
        )
        {
            next_control_ms =
                now_ms +
                CONTROL_PERIOD_MS;
        }

        /* -------------------------------------------------
         * IMU
         * ------------------------------------------------- */

        MPU6050_read_all(
            &ax, &ay, &az,
            &gx, &gy, &gz
        );

        calc_accel_angle();
        complementary_update();

        gyro_angle_y +=
            gy_f *
            CONTROL_DT;

        /* -------------------------------------------------
         * 자세 목표
         * ------------------------------------------------- */

        angle_target = angleY_zero;

        error =
            angleY -
            angle_target;

        /* -------------------------------------------------
         * 넘어짐 처리
         * ------------------------------------------------- */

        if (fabs(error) > FALL_ANGLE)
        {
            motor_set_speed(0.0);

            gy_f  = 0.0;
            gy0_f = 0.0;
            gy_f2 = 0.0;

            if (control_count < 0xFFFFFFFFUL)
            {
                control_count++;
            }

            continue;
        }

        /* -------------------------------------------------
         * 자세 PD
         * ------------------------------------------------- */

        cmd =
            KP * error * 1.07 +
            KD * gy_f2;

        motor_set_speed(cmd);

        /* -------------------------------------------------
         * UART 출력
         * ------------------------------------------------- */

        print_div++;

        if (print_div >= 5)
        {
            double accel_angle;
            double filtered_angle;
            double gyro_rate;
            double angle_error;
            double pd_cmd;
            uint16_t motor_hz;

            print_div = 0;

            /*
             * 가속도 센서 각도
             */
            accel_angle =
                accelAngleY -
                angleY_zero;

            /*
             * 상보필터 각도
             */
            filtered_angle =
                angleY -
                angleY_zero;

            /*
             * 자이로 각속도
             */
            gyro_rate =
                gy_f2;

            /*
             * 자세 오차
             */
            angle_error =
                error;

            /*
             * PD 출력
             */
            pd_cmd =
                cmd;

            /*
             * STEP 주파수
             */
            motor_hz =
                last_hz;

            /*
             * 출력:
             *
             * error,
             * gyro rate,
             * PD cmd,
             * STEP Hz
             */
            sprintf(
                msg,
                "%.2f,%.2f,%.2f,%u\n",
                angle_error,
                gyro_rate,
                pd_cmd,
                motor_hz
            );

            USART0_Transmit_String(msg);
        }

        if (control_count < 0xFFFFFFFFUL)
        {
            control_count++;
        }
    }
}