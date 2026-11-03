import serial
import time
import math
import winsound

# =========================================
# 설정
# =========================================

PORT = "COM3"          # ★ 실제 COM 포트로 수정
BAUD = 38400

REPEAT_COUNT = 5

TARGET_ANGLE = 5.0       # 외란 목표각
STABLE_ERROR = 0.5       # ±0.5° 이내
STABLE_TIME = 1.0        # 1초 유지하면 정상 복귀
MAX_RECOVERY_TIME = 10.0 # 최대 10초 측정


# =========================================
# 시리얼 연결
# =========================================

ser = serial.Serial(PORT, BAUD, timeout=0.2)

time.sleep(2)
ser.reset_input_buffer()


def read_angle():

    while True:

        raw = ser.readline()

        if not raw:
            continue

        try:
            line = raw.decode("utf-8").strip()
            parts = line.split(",")

            # UART 형식:
            # Angle Error, Gyro Rate, PD cmd, STEP Hz
            if len(parts) != 4:
                continue

            return float(parts[0])

        except:
            continue


# =========================================
# 결과 저장
# =========================================

recovery_results = []
trigger_angles = []
max_errors = []


print("==========================================")
print("       5도 외란 복귀시간 실험")
print("==========================================")
print("5도 도달 시 자동으로 측정을 시작합니다.")
print("삐 소리가 나면 바로 손을 떼세요.")
print()
print("복귀 기준 : |Angle Error| <= 0.5도")
print("             1초 이상 유지")
print("반복 횟수 : 5회")
print("==========================================")


for trial in range(1, REPEAT_COUNT + 1):

    print()
    print("==========================================")
    print(f"              {trial}회차")
    print("==========================================")

    # --------------------------------------
    # 1. 정상 상태에서 시작
    # --------------------------------------

    input("로봇이 안정적으로 균형을 잡으면 Enter → ")

    ser.reset_input_buffer()

    print()
    print("이제 로봇을 천천히 한 방향으로 기울이세요.")
    print("5도에 도달하면 자동으로 삐 소리가 납니다.")
    print()

    # --------------------------------------
    # 2. 5도 도달 자동 감지
    # --------------------------------------

    while True:

        angle = read_angle()

        print(
            f"\r현재 Angle Error : {angle:6.2f} deg",
            end="",
            flush=True
        )

        if abs(angle) >= TARGET_ANGLE:

            trigger_angle = angle
            break


    print()
    print()
    print(
        f"★★★★★ 5도 도달 ({trigger_angle:.2f}°) "
        f"- 손 떼세요! ★★★★★"
    )

    # 소리
    winsound.Beep(1500, 200)

    # 이 순간부터 복귀시간 측정
    release_time = time.monotonic()

    stable_start = None
    recovery_time = None
    max_error = abs(trigger_angle)


    # --------------------------------------
    # 3. 복귀 과정 측정
    # --------------------------------------

    while True:

        angle = read_angle()

        now = time.monotonic()
        elapsed = now - release_time

        if abs(angle) > max_error:
            max_error = abs(angle)

        print(
            f"\r시간 : {elapsed:5.2f} s   "
            f"Angle Error : {angle:6.2f} deg",
            end="",
            flush=True
        )


        # -------------------------------
        # ±0.5도 범위 안인지 확인
        # -------------------------------

        if abs(angle) <= STABLE_ERROR:

            if stable_start is None:

                stable_start = now

            elif now - stable_start >= STABLE_TIME:

                # ±0.5도에 처음 진입한 시점을
                # 복귀 완료시간으로 사용
                recovery_time = stable_start - release_time
                break

        else:

            # 중간에 다시 ±0.5도를 벗어나면
            # 안정화 카운트 초기화
            stable_start = None


        # -------------------------------
        # 10초 안에 못 돌아오면 실패
        # -------------------------------

        if elapsed >= MAX_RECOVERY_TIME:
            break


    print()
    print()

    trigger_angles.append(trigger_angle)
    max_errors.append(max_error)


    # --------------------------------------
    # 4. 회차 결과
    # --------------------------------------

    if recovery_time is not None:

        recovery_results.append(recovery_time)

        print(f"===== {trial}회차 결과 =====")
        print(f"외란 시작각 : {trigger_angle:.2f}°")
        print(f"복귀시간    : {recovery_time:.3f} s")
        print(f"최대오차    : {max_error:.2f}°")

    else:

        recovery_results.append(None)

        print(f"===== {trial}회차 결과 =====")
        print(f"외란 시작각 : {trigger_angle:.2f}°")
        print("복귀 실패   : 10초 이내 안정화 실패")


# =========================================
# 최종 결과
# =========================================

ser.close()

valid_results = [
    r for r in recovery_results
    if r is not None
]

print()
print("==========================================")
print("              최종 결과")
print("==========================================")

for i in range(REPEAT_COUNT):

    if recovery_results[i] is not None:

        print(
            f"{i+1}회 : "
            f"외란={trigger_angles[i]:.2f}°, "
            f"복귀={recovery_results[i]:.3f}s, "
            f"최대오차={max_errors[i]:.2f}°"
        )

    else:

        print(
            f"{i+1}회 : "
            f"외란={trigger_angles[i]:.2f}°, "
            f"복귀 실패"
        )


print()
print("------------------------------------------")

if valid_results:

    avg_recovery = sum(valid_results) / len(valid_results)

    print(f"평균 복귀시간 : {avg_recovery:.3f} s")

print(
    f"복귀 성공률   : "
    f"{len(valid_results)}/{REPEAT_COUNT}"
)

print("------------------------------------------")