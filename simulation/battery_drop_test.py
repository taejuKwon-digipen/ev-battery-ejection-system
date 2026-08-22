"""
battery_drop_test.py
3번 파트 체크리스트 "제어 테스트" 검증용 스크립트

실행 전:
  1) CoppeliaSim 실행 + ev_ejection.ttt 씬 열기 (Play 는 누르지 않아도 됨)
  2) 이 파일과 ejection_sim.py 를 같은 폴더에 둘 것

실행:
  python battery_drop_test.py

기대 동작:
  시뮬 시작 -> 3초 전진 -> 정차 -> ARMED(분리 대기) -> 사람 확인(Enter) -> 배터리 링크 해제
  -> 배터리 낙하 -> 그 자리에서 정차 유지 (도망가지 않음)

  "정차했다고 바로 떨어뜨리면 다른 차가 밟을 수 있다"는 문제 때문에, 위험 감지 즉시
  eject()를 부르지 않는다. arm()으로 정차 + 분리 준비만 하고, 안전 확인(이 테스트에서는
  Enter 키 입력으로 대신함 - 실차라면 후방 클리어 센서 또는 기계식 릴리즈 레버) 후에만
  confirm_eject()로 실제 분리를 실행한다.
"""

import time

from ejection_sim import EjectionSim

# 낙하로 인정할 최소 하강 높이 (m)
DROP_THRESHOLD_M = 0.03


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

    # --- 안전 확인 (실차: 후방 클리어 센서 또는 기계식 릴리즈 레버) --------------
    print("    배터리를 분리하려면 수동으로 래치를 당겨야 합니다.")
    input("    래치를 당기려면 Enter를 누르세요... ")

    # --- ARMED -> EJECTED ----------------------------------------------------
    print("[5] 분리 확정")
    sim.confirm_eject()

    # 낙하 관찰
    for i in range(6):
        time.sleep(0.5)
        print(f"    t+{(i + 1) * 0.5:.1f}s  z = {sim.battery_height():.4f} m")

    h_after = sim.battery_height()
    dropped = (h_before - h_after) > DROP_THRESHOLD_M

    print(f"[6] 최종 배터리 높이 : {h_after:.4f} m  (하강 {h_before - h_after:.4f} m)")

    if dropped:
        print("    ✅ 낙하 성공 - 링크 해제 매커니즘 정상 동작")
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
