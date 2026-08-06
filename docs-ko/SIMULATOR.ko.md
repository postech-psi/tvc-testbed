# TVC 6-DOF 자세 시뮬레이터 — 인터랙티브 GUI

> 이 문서는 [../SIMULATOR.md](../SIMULATOR.md)의 한글 번역본입니다. 원본이 기준이며, 내용이 어긋날 경우 영문본을 따르세요.

인터랙티브 데스크톱 GUI를 갖춘 Step 2 시뮬레이터입니다. 기체 파라미터, 제어 게인, 목표/외란 조건을 편집하고 **Run Simulation**을 누르면, 자세·각속도·짐벌 응답이 내장 플롯에 즉시 갱신됩니다.

## ROS2 / Gazebo 개발 환경

이 프로젝트의 Phase 3–5 (ROS2, Gazebo SIL 테스트, PX4 SITL 브리징)는 팀 전체가 동일한 환경을 공유하도록 Docker 개발 컨테이너 안에서 돌아갑니다. 시작하려면 [README.ko.md](README.ko.md)를 보세요 — Docker Desktop과 VS Code만 설치돼 있으면 클릭 몇 번이면 됩니다.

## 파일

- `src/tvc_control/tvc_control/physics.py` — 물리 코어 (쿼터니언 운동학, 뉴턴-오일러 동역학, 짐벌 액추에이터 모델, cascaded PID 컨트롤러, 시뮬레이션 드라이버). 단일 진실 공급원(single source of truth)이며, Phase 4 이후의 ROS2 노드들도 이걸 씁니다. 헤드리스 스모크 테스트로 단독 실행도 가능합니다:
  ```
  python3 src/tvc_control/tvc_control/physics.py
  ```
- `tvc_gui.py` — 인터랙티브 Tkinter GUI. 평소에는 이걸 실행하세요:
  ```
  python3 tvc_gui.py
  ```

## 요구 사항

```
pip install -r requirements.txt
```

`tkinter`가 필요하며, Windows/macOS의 표준 Python 설치본에는 보통 함께 들어 있습니다. Debian/Ubuntu 리눅스에서 `python3 -c "import tkinter"`가 실패한다면 다음으로 설치하세요:

```
sudo apt-get install python3-tk
```

## GUI에서 조정할 수 있는 것

**Vehicle Parameters (기체 파라미터)** — 질량, 롤/피치/요 관성, 축방향 TVC 모멘트 암 `L` (짐벌 피벗 → 질량중심, 수정된 물리 모델에 따름 — 측방향 오프셋이 *아닙니다*), 그리고 선택적인 측방향 CM 정렬오차 외란 항 `dx`/`dy`.

**Actuator Limits (액추에이터 한계)** — 최대/최소 추력, 짐벌 기계적 한계(deg), 짐벌 서보 슬루 레이트(deg/s).

**Cascaded PID Gains** — 외측 루프 각도 게인, 내측 루프 각속도 PID 게인, 그리고 각속도 루프 적분기의 안티와인드업 클램프.

**Simulation / Target** — 시뮬레이션 시간, 제어 주기, 목표 롤/피치, 초기 롤/피치 외란 (Step 2의 "롤 3° / 피치 -4° → 목표 0°/+5°" 형태의 테스트 케이스가 기본값입니다).

## 출력 읽는 법

세 개의 플롯이 위아래로 쌓입니다:

1. **Attitude Response (자세 응답)** — 시간에 따른 롤/피치 오일러각, 점선으로 목표 설정값 표시.
2. **Body Angular Velocity (동체 각속도)** — ω_x, ω_y, ω_z (deg/s).
3. **Gimbal Command (짐벌 명령)** — δ₁ (피치 평면), δ₂ (롤 평면) 짐벌 각도, 점선으로 설정된 기계적 한계 표시.

컨트롤 폼 아래의 메트릭 패널은 최종 롤/피치 오차, 2% 대역 정착 시간(피치 기준), 그리고 최대 짐벌 변위와 한계 대비 활용률(%)을 보고합니다 — 짐벌이 포화 중이면(기계적 한계의 90% 초과) 경고도 함께 나오는데, 이건 보통 외란이 너무 크거나 게인을 다시 튜닝해야 한다는 뜻입니다.

## Step 1 / de Lajarte 4장에서 이어지는 물리 노트

- TVC 모멘트 암은 짐벌 피벗에서 기체 질량중심까지의 **축방향** 거리 `L`입니다 (동체 +z 방향) — 측방향 CM 오프셋이 아닙니다. 측방향 오프셋과 축상 추력 벡터의 외적은 1차 근사에서 토크가 0이므로, `dx`/`dy`는 선택적 외란 항으로만 남겨두었습니다.
- 추력축(동체 z) 주위의 롤/요에는 짐벌 권한이 없습니다 — 실제 요 제어는 동축 모터 간의 RPM 차이에서 나오며, 자세 전용인 이 시뮬레이터에는 모델링돼 있지 않습니다.
- 자세는 짐벌 락을 피하기 위해 전 구간에서 쿼터니언으로 전파합니다 (내부적으로는 절대 오일러각을 쓰지 않습니다). 오일러각은 판독/플로팅 용도로만 계산됩니다.
