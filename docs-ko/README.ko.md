# Docker 개발 환경 — TVC VTVL UGRP

> 이 문서는 [../README.md](../README.md)의 한글 번역본입니다. 원본이 기준이며, 내용이 어긋날 경우 영문본을 따르세요.

이 환경은 ROS2 Jazzy + Gazebo Harmonic + PX4 브리지 도구로 구성된 개발 환경으로, 어느 컴퓨터에서 실행하든 — 본인 노트북이든, 팀원 노트북이든, 연구실 워크스테이션이든 — 완전히 동일합니다. 바로 그 동일함이 여기서 Docker를 쓰는 이유 전부입니다. "제 컴퓨터에서는 되는데요"라는 변명이 성립할 수 없게 되니까요. 모두가 같은 컨테이너를 돌리고 있기 때문입니다.

Phase 1/2의 독립 실행형 시뮬레이터(순수 Python/Tkinter GUI, Docker 불필요)를 찾고 계신가요? [SIMULATOR.ko.md](SIMULATOR.ko.md)를 보세요.

## 개념 정리 (Docker를 이미 안다면 건너뛰세요)

계속 등장하는 세 단어가 있는데, 정확히 짚고 갈 가치가 있습니다:

- **이미지(Image)** — 빌드된 읽기 전용 템플릿 (인스턴스가 아니라 클래스라고 생각하면 됩니다). `Dockerfile`로 정의합니다. 이미지를 빌드한다고 뭔가가 실행되는 게 아니라, 그냥 템플릿이 만들어질 뿐입니다.
- **컨테이너(Container)** — 이미지를 실행한 인스턴스 (그 클래스의 인스턴스). 컨테이너는 시작·정지·삭제를 마음대로 해도 되고, 그래도 원본 이미지에는 아무 영향이 없습니다.
- **재빌드(Rebuild)** — Dockerfile의 명령들을 다시 실행해 새 이미지를 만드는 것. Dockerfile 자체가 바뀌었을 때만 필요합니다. 프로젝트 코드를 고치는 건 재빌드가 *필요 없습니다* — Dockerfile을 고쳤을 때만 필요합니다.

**Dev Container**(VS Code의 "Reopen in Container" 버튼이 쓰는 것)는 순수 Docker 위에 얹힌 한 겹의 관례입니다. `.devcontainer/devcontainer.json` 파일이 VS Code에게 어떤 Dockerfile을 빌드할지, 코드를 어느 폴더에 마운트할지, 어떤 에디터 확장을 컨테이너 *안에* 설치할지 알려줍니다. 결과적으로 VS Code의 터미널·IntelliSense·디버거가 전부 컨테이너 안에서 동작하면서도, 파일 편집은 로컬 파일을 다루듯 그대로 할 수 있습니다.

## 빠른 시작

**1. 두 가지를 한 번만 설치합니다:**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) — 설치 중 "install for this user"를 선택하세요 (관리자 권한 불필요)
- [VS Code](https://code.visualstudio.com/) + **Dev Containers** 확장 (`Ctrl+Shift+X` → "Dev Containers" 검색 → Install)

**2. 프로젝트를 엽니다:**
```bash
cd tvc-testbed
code .
```

**3. VS Code가 물어보면 "Reopen in Container"를 클릭합니다** (오른쪽 아래에 알림이 뜹니다).

이때 실제로 일어나는 일: VS Code가 `.devcontainer/devcontainer.json`을 읽고, 그 파일이 가리키는 `Dockerfile`로 Docker가 이미지를 빌드하며(거기 나열된 도구들을 하나씩 내려받습니다), 그 이미지로 컨테이너를 시작한 뒤 VS Code가 그 안에서 자신을 다시 엽니다. 처음에는 모든 레이어를 내려받고 빌드해야 하므로 대략 10분쯤 걸립니다. 그 이후로는 Docker가 결과를 캐시하므로 다시 여는 건 수 초면 끝나고, Dockerfile을 조금 고쳐 전체 재빌드를 하더라도 변경 지점 이후 레이어만 다시 만듭니다.

**4. 잘 됐는지 확인합니다.** VS Code 안에서 터미널을 열고 (`` Ctrl+` ``) — 이 터미널은 호스트가 아니라 컨테이너 안에서 돌고 있습니다 — 다음을 실행하세요:
```bash
ros2 --version      # → ROS 2 release 'jazzy'
gz sim --version    # → Gazebo Sim, version 8.x
```

둘 다 버전을 출력하면 환경이 준비된 겁니다. 이제부터는 평소처럼 코드를 편집하면 되고, 파일은 내 컴퓨터와 컨테이너 사이에서 실시간으로 동기화됩니다 (`devcontainer.json`의 "workspaceMount" 참고 — 바인드 마운트라서, 컨테이너는 복사본이 아니라 에디터가 보고 있는 것과 정확히 같은 디스크상의 파일을 보고 있습니다).

**5. 첫 ROS2 파이프라인을 돌려봅니다.** `src/tvc_demo/`는 최소한의 ROS2 패키지입니다 — 1초에 한 번씩 숫자를 세는 퍼블리셔 노드와, 받은 걸 출력하는 서브스크라이버 노드. 실제 물리 코드가 개입하기 전에 ROS2 파이프라인 전체(빌드 → 실행 → 토픽 → 메시지)가 작동한다는 걸 확인하려고 일부러 넣어둔 것입니다. Phase 4의 실제 시뮬레이터/컨트롤러 노드도 이와 똑같은 패턴을 따르며, 다만 카운터 대신 기체 자세를 다루게 됩니다.

컨테이너 터미널에서:
```bash
colcon build              # src/ 아래 모든 패키지를 컴파일
source install/setup.bash # 이 셸에서 새 패키지들을 사용할 수 있게 함
ros2 launch tvc_demo demo.launch.py
```
`Publishing: N`과 `Received: N` 로그가 번갈아 찍히면 성공입니다. `Ctrl+C`로 정지합니다. `colcon build`는 패키지 소스를 바꾸거나 추가했을 때만 다시 돌리면 되고, `source install/setup.bash`는 *새로* 여는 터미널마다 다시 실행해야 합니다 (터미널 하나하나가 별개의 셸이라, 이전 셸의 설정이 이어지지 않습니다).

## Git & GitHub 설정

이 저장소는 `github.com/postech-psi/tvc-testbed`에 있습니다. git이 컨테이너와 어떻게 상호작용하는지 몇 가지 알아둘 점이 있습니다.

**클론 (최초 1회):**
```bash
git clone https://github.com/postech-psi/tvc-testbed.git
cd tvc-testbed
code .
```

**Git 신원(identity)** — 컨테이너는 호스트 머신의 git 신원(커밋에 기록될 이름/이메일)을 *자동으로 물려받지 않습니다*. 머신마다 한 번씩 설정하세요:
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```
호스트의 일반 터미널에서 실행해도 되고, 컨테이너에 붙은 VS Code 터미널에서 실행해도 됩니다 — 둘은 별개의 환경이니, 실제로 커밋할 쪽에서 실행하세요. 대부분은 호스트 쪽에서 커밋하고(VS Code의 Source Control 패널은 어느 쪽이든 똑같이 동작합니다) 컨테이너 터미널은 ROS2/빌드 명령에만 씁니다.



둘 다 되지만, 처음 설정하는 경우엔 PAT + VS Code 내장 로그인이 가장 간단합니다.

## 무엇이 들어 있고, 왜 들어 있나

| 도구 | 일반적으로 하는 일 | 이 프로젝트에 필요한 이유 | Phase |
|---|---|---|---|
| ROS2 Jazzy | 프로세스("노드") 간에 "토픽"으로 메시지를 주고받는 미들웨어 | 기체 제어 루프를 위해 작성할 노드들이 이 방식으로 통신합니다 | 3 |
| Gazebo Harmonic | 물리 시뮬레이터 | 실제 하드웨어가 존재하기 전에 시뮬레이션된 기체 동역학을 상대로 제어 코드를 시험할 수 있게 합니다 (SIL: Simulation-In-the-Loop) | 4 |
| Micro-XRCE-DDS-Agent | 프로토콜 브리지 | ROS2의 메시지 형식과 PX4 고유 형식(uXRCE-DDS) 사이를 번역해, ROS2 노드가 PX4 비행 컨트롤러에 명령할 수 있게 합니다 | 5 |
| numpy / scipy / matplotlib | 수치 계산 / 플로팅 | `tvc_physics.py`가 이미 쓰고 있는 것과 같은 스택이라, 호스트에서 돌리든 이 컨테이너에서 돌리든 결과가 동일합니다 | 전체 |

여기 있는 것 중 라즈베리파이나 실제 비행 하드웨어를 겨냥한 건 아직 없습니다 (Phase 6) — 그건 실제로 통신할 Pixhawk가 생겼을 때, 배포 전용으로 따로 만드는 더 작은 이미지가 될 겁니다. 지금 그걸 만들어봐야 시험할 방법도 없는 두 번째 물건을 유지보수하는 셈입니다.

**의도적으로 제외한 것:** ARM 크로스 컴파일러 툴체인(`arm-none-eabi-gcc`). 이 툴체인은 Pixhawk 자체의 마이크로컨트롤러용 PX4 *펌웨어*를 컴파일하는 도구로, PX4 SITL(Phase 4–5에서 쓰는 시뮬레이션 버전, 여기 있는 다른 것들과 같은 컴파일러로 빌드됨)이 쓰는 평범한 x86_64 빌드와는 대상이 다릅니다. 나중에 커스텀 펌웨어를 컴파일해야 할 단계가 오면 그때 한 줄 추가하면 되지, 지금 지고 갈 이유는 없습니다.

## 프로젝트 구성

```
Dockerfile             이미지 정의 — 각 명령이 하는 일은 인라인 주석 참고
.devcontainer/          VS Code에게 컨테이너를 어떻게 빌드하고 열지 알려줌
.dockerignore           Docker가 빌드 컨텍스트로 복사하면 안 되는 파일들 (빌드 산출물, .git 등)
```

`.dockerignore` 파일이 중요한 이유는 일반적으로도 알아둘 만합니다. 프로젝트 폴더의 모든 파일은 — 여기서 제외한 것만 빼고 — 어떤 명령도 참조하지 않는 파일까지 포함해서 "빌드 컨텍스트"로 Docker 빌드 프로세스에 전송됩니다. 빌드 산출물이나 `.git` 히스토리가 큰 저장소에서는 이게 아무 이득 없이 빌드를 느리게 만듭니다 — 그래서 제외하는 겁니다.

## 나중에 라이브러리 추가하기

Docker 이미지에 뭔가를 추가하는 일반적인 형태는 언제나 "Dockerfile을 고치고, 재빌드한다"입니다. 흔한 두 가지 경우:

**시스템/ROS2 패키지** (`apt`로 설치) — 해당하는 `apt-get install` 블록에 추가:
```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
```

**Python 패키지** (`pip`으로 설치) — `pip3 install` 줄에 추가:
```dockerfile
RUN pip3 install --break-system-packages --no-cache-dir \
    numpy scipy matplotlib pyyaml jinja2 empy opencv-python
```

그다음 VS Code에서: `Ctrl+Shift+P` → **"Dev Containers: Rebuild Container"**. Docker는 바뀐 레이어와 그 이후만 다시 만들고 — 그 앞의 레이어는 캐시에서 재사용합니다 — 보통 10분짜리 전체 재빌드가 아니라 1~2분이면 끝납니다.

**확정하기 전에 시험해보고 싶다면:** Dockerfile을 건드리지 않고 실행 중인 컨테이너 안에서 바로 패키지를 설치할 수도 있습니다 (`sudo apt-get install ...` 또는 `pip3 install --break-system-packages ...`). 즉시 적용되지만 다음번 컨테이너 재빌드 때 사라집니다 — 영구적으로 추가할 가치가 있는지 판단하기 전에 시험해보기 좋습니다.

## 문제 해결

| 문제 | 아마도 이런 상황 | 해결 |
|---|---|---|
| "Docker daemon not running" | Docker Desktop이 시작되지 않았거나, 아직 시작 중 | Docker Desktop을 실행하고 고래 아이콘의 애니메이션이 멈출 때까지 기다리세요 |
| 빌드가 중간에 실패 | 보통 다운로드 중 네트워크 문제 | "Reopen in Container"를 다시 클릭하세요 — Docker는 마지막으로 성공한 레이어부터 이어서 하지, 처음부터 다시 하지 않습니다 |
| `ros2: command not found` | 컨테이너 안이 아니라 호스트 머신에서 명령을 실행 중 | VS Code 왼쪽 아래 모서리를 확인하세요 — 컨테이너 이름이 보여야 합니다. 로컬 머신 이름이 보이면 접속돼 있지 않은 겁니다 |
| Dockerfile을 고쳤는데 아무것도 안 바뀜 | Dockerfile 수정은 다음 빌드 때만 반영됩니다 | `Ctrl+Shift+P` → "Dev Containers: Rebuild Container" |
| Gazebo GUI 창이 안 뜸 | Windows/Mac에서는 정상입니다 — 거기서는 시뮬레이터가 기본적으로 헤드리스로 돕니다 | 창에 의존하는 대신 `ros2 topic echo <토픽명>`으로 데이터를 확인하세요 |
| **Windows**: "Container failed to start" (WSLg 소켓 오류) | VS Code의 Dev Containers 확장이 리눅스 GUI 앱 표시를 위해 WSL2의 Wayland 소켓을 컨테이너로 포워딩하려 합니다. WSL2에는 유닉스 도메인 소켓이 `\\wsl.localhost` 네트워크 브리지를 통과하지 못하는 알려진 제약이 있어 마운트가 실패합니다. | `Ctrl+Shift+P` → "Preferences: Open User Settings (JSON)" → `"dev.containers.mountWaylandSocket": false` 추가 → 저장 → 컨테이너 재빌드. GUI 소켓 포워딩이 꺼지지만, Windows에서는 Gazebo가 어차피 헤드리스로 돌기 때문에 필요 없습니다. |
| Source Control 패널이 비어 있음 / git이 "detected dubious ownership"이라고 함 | git(v2.35.2 이후)은 파일 소유자가 현재 사용자와 다른 저장소에 대한 작업을 보안 검사 차원에서 거부합니다. 컨테이너의 `postCreateCommand`가 `/workspace`의 소유권을 `ros` 사용자로 바꾸는데, 이게 그 검사에 걸릴 수 있습니다. | 컨테이너 터미널에서 `git config --global --add safe.directory /workspace`를 한 번 실행하고 VS Code 창을 새로고침하세요. |

## 나중에 합류하는 팀원에게

위의 빠른 시작 절차가 모두에게 똑같이 적용됩니다. 이미지는 머신당 한 번만 빌드하면 되고 그 뒤로는 Docker가 캐시하므로, 두 번째 사람의 첫 빌드도 빠릅니다 (2~3분) — 느린 첫 빌드는 이 이미지를 세상에서 처음 빌드하는 머신에서 딱 한 번만 일어납니다.
