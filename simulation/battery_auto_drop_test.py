"""
battery_auto_drop_test.py
"가만히 있으면 자동으로 분리" 시나리오 검증용 스크립트 (battery_drop_test.py의 자동 버전)

battery_drop_test.py 는 ARMED 상태에서 사람이 Enter를 눌러야 분리가 진행된다
(실차라면 후방 클리어 센서 또는 기계식 릴리즈 레버가 그 역할).
이 스크립트는 그 사람 입력을 없애고, "차가 정차한 채 일정 시간 가만히 있으면
주변이 안전하다고 자동 판단해 분리한다"는 시나리오를 검증한다.

실행 전:
  1) CoppeliaSim 실행 + ev_ejection.ttt 씬 열기 (Play 는 누르지 않아도 됨)
  2) 이 파일과 ejection_sim.py 를 같은 폴더에 둘 것

실행:
  python battery_auto_drop_test.py

기대 동작:
  시뮬 시작 -> 3초 전진 -> 감속 -> 정차 -> ARMED(분리 대기)
  -> 정차 상태가 SAFETY_DELAY_S 초 동안 유지되는지 자동 확인
  -> 자동으로 배터리 링크 해제 -> 배터리 낙하 -> 그 자리에서 정차 유지
"""

import time

from ejection_sim import EjectionSim

# 낙하로 인정할 최소 하강 높이 (m)
DROP_THRESHOLD_M = 0.03

# 정차 후 이 시간(초) 동안 계속 가만히 있으면 "주변이 안전하다"고 자동 판단한다.
# 실차라면 이 시간 대신 후방 클리어 초음파 센서 값을 확인해야 한다.
SAFETY_DELAY_S = 2.0


def main() -> int:
    sim = EjectionSim()

    try:
        sim.connect()
    except Exception as e:
        print("접속 실패:", e)
        print("→ CoppeliaSim 이 실행 중인지, 씬이 열려 있는지 확인하세요.")
        return 1

    # 이전 테스트 잔재를 지우고 깨끗한 상태에서 시작
    sim.reset()

    h_before = sim.battery_height()
    print(f"[1] 초기 배터리 높이 : {h_before:.4f} m")

    # --- NORMAL: 정상 주행 -------------------------------------------------
    print("[2] 정상 주행 3초...")
    sim.forward(1.5)
    time.sleep(3.0)

    # --- WARNING: 감속 -----------------------------------------------------
    print("[3] 경고 상태 - 감속 1초...")
    sim.slow()
    time.sleep(1.0)

    # --- STOPPED -> ARMED ----------------------------------------------------
    print("[4] 위험 감지 - 정차 후 분리 대기(ARMED)")
    sim.arm()

    # --- 자동 안전 확인 (사람 확인 없음) --------------------------------------
    print(f"    사람 확인 없이, 정차 상태가 {SAFETY_DELAY_S:.1f}초 유지되는지 자동 확인합니다...")
    ejected = sim.auto_confirm_eject(safety_delay=SAFETY_DELAY_S)

    if not ejected:
        print("    ❌ 자동 분리 실패 - 정차 상태가 유지되지 않았거나 ARMED가 아니었습니다.")
        sim.stop()
        return 2

    # 낙하 관찰
    for i in range(6):
        time.sleep(0.5)
        print(f"    t+{(i + 1) * 0.5:.1f}s  z = {sim.battery_height():.4f} m")

    h_after = sim.battery_height()
    dropped = (h_before - h_after) > DROP_THRESHOLD_M

    print(f"[6] 최종 배터리 높이 : {h_after:.4f} m  (하강 {h_before - h_after:.4f} m)")

    if dropped:
        print("    ✅ 자동 낙하 성공 - 정차 유지 감지 기반 자동 분리 정상 동작")
    else:
        print("    ❌ 낙하 실패 - 아래를 확인하세요")
        print("       - EV_Battery 의 'Body is static' 체크가 풀려 있는지")
        print("       - EV_Battery 가 EV_Battery_Link 의 자식으로 되어 있는지")
        print("       - 배터리가 차체나 바닥에 끼어 있지 않은지")

    # --- 분리 후 정차 유지 ---------------------------------------------------
    print("[7] 분리 완료 - 도망가지 않고 그 자리에 정차 유지")
    sim.halt()

    time.sleep(1.0)
    sim.stop()
    print("[8] 테스트 종료")
    return 0 if dropped else 2


if __name__ == "__main__":
    raise SystemExit(main())
