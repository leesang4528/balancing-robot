import serial
import time
import math

# =========================================
# 설정
# =========================================

PORT = "COM3"          # ★ 네 COM 포트로 변경
BAUD = 38400

MEASURE_TIME = 10      # 1회 측정시간 : 10초
REPEAT_COUNT = 5       # 반복 횟수 : 5회


# =========================================
# 시리얼 연결
# =========================================

ser = serial.Serial(PORT, BAUD, timeout=0.2)

time.sleep(2)
ser.reset_input_buffer()

print("===================================")
print(" 외란 없는 밸런싱 성능 측정")
print(" 10초 측정 × 5회")
print("===================================")


rms_results = []
mae_results = []
max_results = []


# =========================================
# 반복 실험
# =========================================

for trial in range(1, REPEAT_COUNT + 1):

    print()
    print(f"===== {trial}회차 준비 =====")
    print("로봇이 안정적으로 균형을 잡은 뒤 Enter를 누르세요.")
    input()

    ser.reset_input_buffer()

    errors = []

    print(f"{trial}회차 측정 시작!")

    start_time = time.time()

    while time.time() - start_time < MEASURE_TIME:

        raw = ser.readline()

        if not raw:
            continue

        try:
            line = raw.decode("utf-8").strip()

            parts = line.split(",")

            # 데이터가 4개가 아니면 무시
            if len(parts) != 4:
                continue

            # 첫 번째 값 = Angle Error
            angle_error = float(parts[0])

            errors.append(angle_error)

            print(f"Angle Error = {angle_error:.2f} deg")

        except:
            continue


    # =====================================
    # 한 회차 결과 계산
    # =====================================

    if len(errors) > 0:

        n = len(errors)

        # 평균 절대 오차
        mae = sum(abs(e) for e in errors) / n

        # RMS 오차
        rms = math.sqrt(
            sum(e * e for e in errors) / n
        )

        # 최대 절대 오차
        max_error = max(abs(e) for e in errors)


        mae_results.append(mae)
        rms_results.append(rms)
        max_results.append(max_error)


        print()
        print(f"===== {trial}회차 결과 =====")
        print(f"샘플 수     : {n}")
        print(f"MAE         : {mae:.3f} deg")
        print(f"RMS Error   : {rms:.3f} deg")
        print(f"Max Error   : {max_error:.3f} deg")

    else:

        print("데이터를 받지 못했습니다.")


# =========================================
# 전체 5회 평균
# =========================================

ser.close()

if len(rms_results) > 0:

    avg_mae = sum(mae_results) / len(mae_results)
    avg_rms = sum(rms_results) / len(rms_results)
    avg_max = sum(max_results) / len(max_results)

    print()
    print("===================================")
    print("       최종 5회 평균 결과")
    print("===================================")

    for i in range(len(rms_results)):

        print(
            f"{i+1}회 : "
            f"MAE={mae_results[i]:.3f}°, "
            f"RMS={rms_results[i]:.3f}°, "
            f"MAX={max_results[i]:.3f}°"
        )

    print()
    print("-----------------------------------")
    print(f"평균 MAE       : {avg_mae:.3f} deg")
    print(f"평균 RMS Error : {avg_rms:.3f} deg")
    print(f"평균 Max Error : {avg_max:.3f} deg")
    print("-----------------------------------")

else:

    print("유효한 측정 결과가 없습니다.")
