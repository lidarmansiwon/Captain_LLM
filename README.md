# AI Captain System (ROS2 + LLM + VRX)

## 개요
이 프로젝트는 **ROS2**, **LLM(OpenAI Chat API)**, **VRX 시뮬레이터**를 활용해  
무인선(USV)의 **AI Captain** 시스템을 실험하기 위한 구조를 제공합니다.  

- 자연어 명령 → LLM → 구조화된 JSON(orders) → Captain 노드  
- Captain이 의도 해석, 상태 관리, 명령 큐잉 → SubCaptain → Server  
- Server는 실제 실행/사람 확인을 담당  
- VRX thruster 제어 및 항법(Odometry) 감시 기능 포함

---

## 주요 컴포넌트

### 1. rosgpt (게이트웨이)
- Flask + ROS2 노드
- 자연어 입력(`text_command`)을 받아 LLM 호출
- LLM 응답을 **온톨로지 기반 JSON**으로 변환 후 `/voice_cmd` 퍼블리시
- `Mission Start` 전에는 다른 명령을 무시
- 대화 히스토리 관리(`client_id` 기반)

### 2. Captain Node
- `/voice_cmd` 구독 → 미션 시작/명령 JSON 처리
- `ShipStatus` 관리 및 주기 퍼블리시(`/ship/status`)
  - `navigation`, `thruster`, `operation`, `note`
- 명령 큐잉 → `/captain/order` 발행
- `/subcaptain/response` 수신 후 상태 업데이트
- 출항(5)은 `navigation && thruster == True`일 때만 허용
- 부족 항목이 있으면 자동으로 재검사 명령 enqueue

### 3. SubCaptain Node
- `/captain/order` 구독
- `/server/execute_order` 서비스 호출
- 결과를 `/subcaptain/response` 퍼블리시

### 4. Server Node
- `/server/execute_order` 서비스 서버
- 주문 처리:
  - 3: Thruster check → 사용자 입력 True/False
  - 4: Navigation check → `NavGuardian` 상태 기반 or 사용자 확인
  - 5: Departure → 사용자 입력 승인 여부
- "확인하는 사람" 역할

### 5. NavGuardian Node
- `/agent_0/odom`(nav_msgs/Odometry) 감시
- 오돔이 끊기면 `/nav/health=False` 발행, 들어오면 True
- Captain은 이를 받아 `ShipStatus.navigation` 자동 갱신
- 운항 중 오돔 끊기면 `operation=False` 로 자동 전환
- 옵션: 네비 끊김 시 rosgpt에 알림 → LLM이 재검사 명령(JSON) 생성

### 6. ThrusterCommander Node
- `/ship/status` 구독
- 20Hz 주기로 좌/우 추진기 토픽 발행
  - 조건 충족(`navigation && thruster && operation`) → `180.0`
  - 아니면 `0.0`
- 토픽:
  - `/agent0/thrusters/left/thrust` (std_msgs/Float64)  
  - `/agent0/thrusters/right/thrust` (std_msgs/Float64)

---

## 메시지/서비스 정의

### captain_interfaces/msg/ShipStatus.msg
```msg
bool navigation
bool thruster
bool operation
string note
