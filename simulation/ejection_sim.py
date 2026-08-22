"""
ejection_sim.py
CoppeliaSim EV 배터리 분리 시뮬레이션 제어 모듈

사용 전 준비:
  1) pip install coppeliasim-zmqremoteapi-client
  2) CoppeliaSim 실행 + ev_ejection.ttt 씬 열기
  3) 씬에 아래 이름의 오브젝트가 있어야 함
       /PioneerP3DX            (차체)
       /PioneerP3DX/leftMotor  (좌측 바퀴 조인트)
       /PioneerP3DX/rightMotor (우측 바퀴 조인트)
       /EV_Battery_Link        (포스센서 = 락킹 링크)
       /EV_Battery             (배터리 블록, dynamic + respondable)

데이터 담당자용 사용 예:
    from ejection_sim import EjectionSim

    sim = EjectionSim()
    sim.connect()
    sim.start()
    sim.drive(1.5, 1.5)

    if danger_flag:
        sim.arm()               # 정차 + 분리 준비 (아직 분리하지 않음, 아두이노 STOPPED->ARMED와 대응)
        if area_clear_flag:     # 후방/주변 안전 확인(초음파 등) 또는 사람의 확인
            sim.confirm_eject() # 실제 분리 실행

  "정차했다고 바로 떨어뜨리면 다른 차가 밟을 수 있다"는 문제 때문에, danger_flag 하나로
  바로 eject()를 부르지 않는다. arm()으로 준비만 하고, 안전이 확인됐을 때만
  confirm_eject()를 불러야 실제로 분리된다. eject()를 직접 호출하는 건 안전 확인
  절차를 건너뛰는 저수준 강제 분리이므로 데모/테스트 용도로만 사용할 것.
"""

from __future__ import annotations

import math
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ---------------------------------------------------------------------------
# 씬 오브젝트 경로 (CoppeliaSim에서 지은 이름과 반드시 일치해야 함)
# check_scene.py 를 실행하면 실제 경로를 확인할 수 있음
# ---------------------------------------------------------------------------
ROBOT_PATH = "/PioneerP3DX"
LEFT_MOTOR_PATH = "/PioneerP3DX/leftMotor"
RIGHT_MOTOR_PATH = "/PioneerP3DX/rightMotor"
BATTERY_PATH = "/EV_Battery"
LINK_PATH = "/EV_Battery_Link"

# 주행 속도 기본값 (rad/s)
CRUISE_SPEED = 1.5
WARNING_SPEED = 0.5

# 분리 직후 "뒤로 포물선을 그리며 튕겨나가는" 연출 파라미터
EJECT_BACK_DISTANCE_M = 0.15  # 뒤로 이동하는 거리
EJECT_ARC_HEIGHT_M = 0.05     # 위로 튀는 높이(포물선 정점)
EJECT_ARC_DURATION_S = 0.4    # 연출 지속 시간
EJECT_ARC_STEPS = 12          # 연출 프레임 수 (많을수록 부드러움)


def connect_sim(host: str = "localhost", port: int = 23000):
    """CoppeliaSim ZMQ Remote API 서버에 접속해 (client, sim) 을 반환."""
    client = RemoteAPIClient(host=host, port=port)
    try:
        # CoppeliaSim 4.6 이상 / 최신 클라이언트
        sim = client.require("sim")
    except Exception:
        # 구버전 클라이언트 호환
        sim = client.getObject("sim")
    return client, sim


class EjectionSim:
    """CoppeliaSim 안의 차체 주행 + 배터리 분리를 제어한다."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 23000,
        robot_path: str = ROBOT_PATH,
        left_motor_path: str = LEFT_MOTOR_PATH,
        right_motor_path: str = RIGHT_MOTOR_PATH,
        battery_path: str = BATTERY_PATH,
        link_path: str = LINK_PATH,
        use_force_sensor_break: bool = False,
    ):
        self.host = host
        self.port = port
        self.paths = {
            "robot": robot_path,
            "left": left_motor_path,
            "right": right_motor_path,
            "battery": battery_path,
            "link": link_path,
        }
        # True  -> sim.breakForceSensor() 로 링크를 끊음
        # False -> sim.setObjectParent(..., -1) 로 계층에서 분리 (더 안정적, 권장)
        self.use_force_sensor_break = use_force_sensor_break

        self.client = None
        self.sim = None
        self.handles: dict[str, int] = {}
        self._ejected = False
        self._armed = False

    # -- 연결 ---------------------------------------------------------------

    def connect(self) -> "EjectionSim":
        """시뮬레이터에 접속하고 오브젝트 핸들을 모두 확보한다."""
        self.client, self.sim = connect_sim(self.host, self.port)

        missing = []
        for key, path in self.paths.items():
            try:
                self.handles[key] = self.sim.getObject(path)
            except Exception:
                self.handles[key] = None
                # link(포스센서)는 없어도 동작 가능하므로 필수 목록에서 제외
                if key != "link":
                    missing.append(path)

        if missing:
            raise RuntimeError(
                "씬에서 다음 오브젝트를 찾지 못했습니다: "
                + ", ".join(missing)
                + "\n→ check_scene.py 를 실행해 실제 이름을 확인하세요."
            )

        print(f"[EjectionSim] 접속 완료 ({self.host}:{self.port})")
        return self

    def _require_connected(self):
        if self.sim is None:
            raise RuntimeError("connect() 를 먼저 호출하세요.")

    # -- 시뮬레이션 수명주기 -------------------------------------------------

    def start(self):
        """시뮬레이션 시작 (이미 실행 중이면 그대로 둔다)."""
        self._require_connected()
        if self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()
            time.sleep(0.3)  # 물리 엔진 초기화 대기
        self._ejected = False
        self._armed = False
        print("[EjectionSim] 시뮬레이션 시작")

    def stop(self):
        """시뮬레이션 정지. 씬은 자동으로 초기 상태로 복구된다."""
        self._require_connected()
        self.sim.stopSimulation()
        # 완전히 멈출 때까지 대기
        for _ in range(100):
            if self.sim.getSimulationState() == self.sim.simulation_stopped:
                break
            time.sleep(0.05)
        self._ejected = False
        self._armed = False
        print("[EjectionSim] 시뮬레이션 정지 (씬 원상 복구)")

    def reset(self):
        """정지 후 재시작 = 배터리 재장착. 아두이노 리셋 버튼과 같은 역할."""
        self.stop()
        time.sleep(0.3)
        self.start()

    # -- 주행 제어 -----------------------------------------------------------

    def drive(self, left_speed: float, right_speed: float):
        """좌/우 바퀴 목표 속도 설정 (rad/s). 분리 후에는 무시된다."""
        self._require_connected()
        self.sim.setJointTargetVelocity(self.handles["left"], float(left_speed))
        self.sim.setJointTargetVelocity(self.handles["right"], float(right_speed))

    def forward(self, speed: float = CRUISE_SPEED):
        self.drive(speed, speed)

    def slow(self):
        """WARNING 상태용 감속 주행."""
        self.drive(WARNING_SPEED, WARNING_SPEED)

    def halt(self):
        """STOPPED 상태용 정차."""
        self.drive(0.0, 0.0)

    # -- 배터리 분리 ---------------------------------------------------------

    def arm(self):
        """
        분리 준비(ARMED) 상태로 진입한다. 차량을 정차시키기만 하고 아직 분리하지 않는다.
        아두이노 FSM의 STOPPED -> ARMED 전이와 대응. "정차했다고 바로 떨어뜨리면
        다른 차가 밟을 수 있다"는 문제 때문에, 실제 분리는 반드시 confirm_eject()를
        따로 호출해야 실행되도록 분리해뒀다.
        """
        self._require_connected()
        self.halt()
        self._armed = True
        print("[EjectionSim] ARMED - 분리 대기 중 (confirm_eject() 호출 시 실제 분리)")

    @property
    def armed(self) -> bool:
        return self._armed

    def confirm_eject(self) -> bool:
        """
        ARMED 상태에서 안전 확인(후방 클리어 센서 또는 사람의 확인) 후 실제 분리를 실행한다.
        arm() 없이 호출되면 실수로 즉시 분리되는 것을 막기 위해 무시하고 False를 반환한다.
        """
        if not self._armed:
            print("[EjectionSim] confirm_eject() 무시됨 - 먼저 arm()으로 분리를 준비하세요.")
            return False
        return self.eject()

    def eject(self) -> bool:
        """
        차체를 정차시키고 배터리 링크를 해제하는 저수준 실행 함수.
        안전 확인 절차(arm/confirm_eject)를 건너뛰므로 위험 감지 즉시 직접 호출하지
        말 것 - 팀 통합 코드에서는 arm() -> confirm_eject() 순서로 사용한다.
        이미 분리된 상태면 아무 것도 하지 않고 False 를 반환한다.
        """
        self._require_connected()
        if self._ejected:
            return False

        self.halt()  # FSM: 정차 -> 분리 순서 유지
        time.sleep(0.2)

        link = self.handles.get("link")
        if self.use_force_sensor_break and link is not None:
            self.sim.breakForceSensor(link)
        else:
            # 계층에서 떼어내면 물리 링크가 재구성되면서 자유낙하한다.
            # 세 번째 인자 True = 현재 위치를 유지한 채로 분리
            self.sim.setObjectParent(self.handles["battery"], -1, True)

        self._ejected = True
        self._armed = False
        self._launch_backward()
        print("[EjectionSim] ⚠ 배터리 분리(EJECT) 실행")
        return True

    def _launch_backward(
        self,
        distance: float = EJECT_BACK_DISTANCE_M,
        arc_height: float = EJECT_ARC_HEIGHT_M,
        duration: float = EJECT_ARC_DURATION_S,
        steps: int = EJECT_ARC_STEPS,
    ):
        """
        분리 직후 배터리가 차체 뒤쪽으로 포물선을 그리며 튕겨나가는 것처럼 보이게
        짧게 위치를 직접 스크립트로 움직인다. "떨어진 게 티가 안 난다"는 문제 때문에
        추가함 - 정확한 물리 임펄스 계산 대신, 눈에 확실히 보이는 짧은 연출로 처리한다.
        연출이 끝나면(포물선 정점을 지나 원래 높이로 복귀) 이후 낙하는 물리엔진이 이어받는다.
        """
        battery = self.handles["battery"]
        robot = self.handles["robot"]

        # 로봇의 로컬 -X(후방), +Z(위) 축을 월드좌표로 변환.
        # getObjectMatrix는 12개 값의 3x4 행렬(row-major)을 반환하며,
        # 각 열이 그 물체의 로컬 X/Y/Z축을 월드 좌표계에서 표현한다.
        m = self.sim.getObjectMatrix(robot, -1)
        rear = (-m[0], -m[4], -m[8])
        up = (m[2], m[6], m[10])

        x0, y0, z0 = self.sim.getObjectPosition(battery, -1)
        dt = duration / steps

        for i in range(1, steps + 1):
            t = i / steps
            back = distance * t
            up_offset = arc_height * math.sin(math.pi * t)  # 0 -> 정점 -> 0
            pos = [
                x0 + rear[0] * back + up[0] * up_offset,
                y0 + rear[1] * back + up[1] * up_offset,
                z0 + rear[2] * back + up[2] * up_offset,
            ]
            self.sim.setObjectPosition(battery, -1, pos)
            time.sleep(dt)

    def escape(self, speed: float = 3.0, duration: float = 2.0):
        """배터리를 버린 뒤 현장에서 급히 이탈하는 연출."""
        if not self._ejected:
            self.eject()
        time.sleep(0.3)
        self.forward(speed)
        time.sleep(duration)
        self.halt()

    # -- 상태 조회 -----------------------------------------------------------

    @property
    def ejected(self) -> bool:
        return self._ejected

    def battery_height(self) -> float:
        """배터리 블록의 월드 기준 z 높이(m). 낙하 여부 판정용."""
        self._require_connected()
        h = self.handles["battery"]
        try:
            pos = self.sim.getObjectPosition(h, self.sim.handle_world)
        except Exception:
            pos = self.sim.getObjectPosition(h, -1)
        return pos[2]

    # -- with 문 지원 --------------------------------------------------------

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        except Exception:
            pass
