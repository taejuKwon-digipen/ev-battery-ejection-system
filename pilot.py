import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import random

# --- 1. 알고리즘 설정값 ---
WINDOW_SIZE = 5       # 이동 평균 필터 윈도우 크기 (N)
TEMP_THRESHOLD = 1.5  # 열폭주 판단 임계 기울기 (alpha)

# 데이터 저장 큐
raw_temps = deque([25.0]*100, maxlen=100) 
filtered_temps = deque([25.0]*100, maxlen=100)

# 가상 시나리오용 초기 온도
current_sim_temp = 25.0

# --- 2. 발표용 대시보드 UI 세팅 ---
# 스타일을 깔끔하게 지정 (발표 시 가독성 향상)
plt.style.use('seaborn-v0_8-darkgrid')
fig, ax = plt.subplots(figsize=(10, 6))
fig.canvas.manager.set_window_title('EV Battery Active Ejection System - HIL Simulator')

line1, = ax.plot(raw_temps, label='Raw Sensor (Noise)', color='gray', alpha=0.4, linestyle='--')
line2, = ax.plot(filtered_temps, label='BMS Filtered (Moving Avg)', color='#d62728', linewidth=2.5)

ax.set_ylim(20, 90)
ax.set_title("EV Battery Thermal Runaway Detection & Ejection System", fontsize=16, fontweight='bold')
ax.set_ylabel("Temperature ($^\circ C$)", fontsize=12)
ax.set_xlabel("Time (Timeline)", fontsize=12)
ax.legend(loc='upper left', fontsize=11)

# 상태 표시 텍스트 박스
status_text = ax.text(0.5, 0.9, 'STATUS: [NORMAL] Driving', 
                       transform=ax.transAxes, color='black', 
                       bbox=dict(facecolor='#2ca02c', alpha=0.8, edgecolor='none', boxstyle='round,pad=0.5'),
                       ha='center', fontsize=14, fontweight='bold')

# --- 3. 실시간 시나리오 업데이트 함수 ---
def update(frame):
    global current_sim_temp
    
    # [시나리오 구성] 시간에 따른 배터리 상태 변화 연출
    noise = random.uniform(-0.8, 0.8) # 주행 진동 노이즈
    
    if frame < 50:
        # Phase 1 (0~5초): 정상 주행 상태 (25도 유지)
        current_sim_temp = 25.0 + noise
    elif frame < 90:
        # Phase 2 (5~9초): 미세 발열 시작 (초당 0.2도씩 상승)
        current_sim_temp += 0.2 + noise
        status_text.set_text('STATUS: [WARNING] Abnormal Heat')
        status_text.set_bbox(dict(facecolor='#ff7f0e', alpha=0.8, edgecolor='none', boxstyle='round,pad=0.5'))
    elif frame < 120:
        # Phase 3 (9~12초): 열폭주 발생! (초당 2.0도씩 급상승)
        current_sim_temp += 2.0 + noise
    else:
        # Phase 4 (12초 이후): 배터리 투하 완료 (센서 연결 끊김, 실온 복귀)
        current_sim_temp = 25.0 + noise
        
    raw_temps.append(current_sim_temp)

    # 수학적 필터링 적용 (이동 평균)
    recent_N = list(raw_temps)[-WINDOW_SIZE:]
    avg_temp = sum(recent_N) / len(recent_N)
    filtered_temps.append(avg_temp)

    # 미분값(온도 상승률) 계산
    temp_diff = filtered_temps[-1] - filtered_temps[-2]

    # 시스템 작동 판단 로직
    if temp_diff > TEMP_THRESHOLD and frame < 120:
        # 열폭주 감지 및 투하 명령!
        status_text.set_text('STATUS: [DANGER] EJECT BATTERY!!')
        status_text.set_color('white')
        status_text.set_bbox(dict(facecolor='#d62728', alpha=0.9, edgecolor='none', boxstyle='round,pad=0.5'))
        ax.set_facecolor('#ffe6e6') # 배경을 붉게 깜빡임
    elif frame >= 120:
        # 투하 완료 상태
        status_text.set_text('STATUS: [SAFE] Battery Ejected')
        status_text.set_color('white')
        status_text.set_bbox(dict(facecolor='#1f77b4', alpha=0.9, edgecolor='none', boxstyle='round,pad=0.5'))
        ax.set_facecolor('white')
    elif frame < 90:
        ax.set_facecolor('white')
        status_text.set_color('black')

    line1.set_ydata(raw_temps)
    line2.set_ydata(filtered_temps)
    
    return line1, line2, status_text

# --- 4. 애니메이션 실행 (100ms 간격) ---
ani = animation.FuncAnimation(fig, update, frames=200, interval=100, blit=False)
plt.show()