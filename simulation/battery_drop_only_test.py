"""
battery_drop_only_test.py
주행(전진/감속) 없이, 연결하자마자 배터리 분리(낙하)만 바로 확인하는 스크립트.

실행 전:
  1) CoppeliaSim 실행 + ev_ejection.ttt 씬 열기 (Play 는 누르지 않아도 됨)
  2) 이 파일과 ejection_sim.py 를 같은 폴더에 둘 것

실행:
  python battery_drop_only_test.py

기대 동작:
  시뮬 시작 -> 바로 배터리 링크 해제 -> 배터리 낙하 확인 -> 종료
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

    print("[2] 배터리 분리(EJECT) 실행")
    sim.eject()

    # 낙하 관찰
    for i in range(6):
        time.sleep(0.5)
        print(f"    t+{(i + 1) * 0.5:.1f}s  z = {sim.battery_height():.4f} m")

    h_after = sim.battery_height()
    dropped = (h_before - h_after) > DROP_THRESHOLD_M

    print(f"[3] 최종 배터리 높이 : {h_after:.4f} m  (하강 {h_before - h_after:.4f} m)")

    if dropped:
        print("    ✅ 낙하 성공 - 링크 해제 매커니즘 정상 동작")
    else:
        print("    ❌ 낙하 실패 - 아래를 확인하세요")
        print("       - EV_Battery 의 'Body is static' 체크가 풀려 있는지")
        print("       - EV_Battery 가 EV_Battery_Link 의 자식으로 되어 있는지")
        print("       - 배터리가 차체나 바닥에 끼어 있지 않은지")

    time.sleep(1.0)
    sim.stop()
    print("[4] 테스트 종료")
    return 0 if dropped else 2


if __name__ == "__main__":
    raise SystemExit(main())
