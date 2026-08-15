"""
check_scene.py
CoppeliaSim 연결 상태와 씬에 있는 오브젝트 경로를 전부 출력한다.

"오브젝트를 못 찾겠다"는 에러가 났을 때 가장 먼저 실행할 것.
여기서 나온 경로를 ejection_sim.py 상단의 상수에 그대로 복사해 넣으면 된다.

실행:
  python check_scene.py
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ejection_sim.py 가 찾는 경로들
EXPECTED = [
    "/PioneerP3DX",
    "/PioneerP3DX/leftMotor",
    "/PioneerP3DX/rightMotor",
    "/EV_Battery_Link",
    "/EV_Battery",
]


def connect():
    client = RemoteAPIClient()
    try:
        sim = client.require("sim")
    except Exception:
        sim = client.getObject("sim")
    return client, sim


def object_path(sim, handle):
    """오브젝트의 전체 경로 문자열을 최대한 얻어온다."""
    for opt in (5, 2, 1):
        try:
            return sim.getObjectAlias(handle, opt)
        except Exception:
            continue
    try:
        return sim.getObjectAlias(handle)
    except Exception:
        return f"<handle {handle}>"


def list_all(sim):
    """씬의 모든 오브젝트 핸들을 수집한다."""
    try:
        return sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    except Exception:
        pass

    handles = []
    i = 0
    while True:
        try:
            h = sim.getObjects(i, sim.handle_all)
        except Exception:
            break
        if h is None or h < 0:
            break
        handles.append(h)
        i += 1
    return handles


def main() -> int:
    try:
        client, sim = connect()
    except Exception as e:
        print("❌ CoppeliaSim 접속 실패:", e)
        print()
        print("확인할 것:")
        print("  1. CoppeliaSim 이 실행 중인가?")
        print("  2. pip install -U coppeliasim-zmqremoteapi-client 했는가?")
        print("  3. 방화벽이 localhost:23000 을 막고 있지 않은가?")
        return 1

    print("✅ CoppeliaSim 접속 성공")
    try:
        print("   시뮬레이터 버전:", sim.getInt32Param(sim.intparam_program_full_version))
    except Exception:
        pass
    print()

    print("=== 씬에 존재하는 오브젝트 ===")
    handles = list_all(sim)
    if not handles:
        print("  (비어 있음 - 씬을 먼저 열어주세요)")
    for h in handles:
        print(f"  {object_path(sim, h)}")
    print()

    print("=== ejection_sim.py 가 필요로 하는 경로 점검 ===")
    ok = True
    for path in EXPECTED:
        try:
            sim.getObject(path)
            print(f"  ✅ {path}")
        except Exception:
            ok = False
            print(f"  ❌ {path}  ← 없음 (이름 오타이거나 아직 안 만듦)")

    print()
    if ok:
        print("전부 준비됐습니다. battery_drop_test.py 를 실행하세요.")
    else:
        print("위에서 ❌ 인 항목을 CoppeliaSim 에서 만들거나 이름을 수정하세요.")
        print("(계층 창에서 이름을 더블클릭하면 변경 가능)")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
