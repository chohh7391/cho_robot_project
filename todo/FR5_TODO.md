# FR5 — ROS2 통합 계획 (옵션 A: 벤더 ros2_control 하드웨어 인터페이스 재사용/확장)

> 목적: FAIRINO FR5를 `cho_robot_project`에 **UR 수직구조와 동일한 방식**으로 추가한다.
> cho의 커스텀 컨트롤러(`task_space_ik` / `joint_space_position`)를 ros2_control 위에서 구동하고,
> 실기는 **벤더가 제공하는 `fairino_hardware/FairinoHardwareInterface`(libfairino C++ SDK 기반)**로 제어한다.
>
> UR이 `ur_robot_driver`를 재사용하듯, FR5는 `fairino_hardware`를 재사용한다.

---

## 0. 핵심 판단 (수정됨 — 로컬 소스 확인)

- **정정**: 앞서 "벤더엔 ros2_control HW 인터페이스가 없다"고 했으나 **틀렸다**(GitHub README 요약만 신뢰한 결과).
  로컬 `~/Downloads/frcobot_ros2/`를 직접 확인한 결과, **진짜 `hardware_interface::SystemInterface`가 존재**한다:
  - 클래스 `fairino_hardware::FairinoHardwareInterface`, 플러그인 `fairino_hardware/FairinoHardwareInterface`
    (`fairino_hardware_v3_9_9/fairino_hardware.xml`, `PLUGINLIB_EXPORT_CLASS ...`).
  - **`libfairino`(FR 공식 C++ SDK)** 기반: `#include "libfairino/include/robot.h"`, `FRRobot`.
  - `write()`가 매 사이클 **`ServoJ(cmd, ext, 0,0, 0.008, 0,0)`** 스트리밍(rad→deg). `read()`는 `GetActualJointPosDegree`(deg→rad).
  - 버전: **최신 `fairino_hardware_v3_9_9`** 사용(최고 넘버; `master`도 velocity 미배선 동일). 펌웨어는
    사용자가 로봇에서 업데이트해 맞춤 → 버전 매칭 비이슈.
- **그래서 옵션 A는 "직접 작성"이 아니라 "벤더 인터페이스 재사용/확장"으로 바뀐다.** 새 엔지니어링이 대폭 줄어든다.
- MoveIt2 config들의 `*.ros2_control.xacro`(예: `fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro`)는
  기본 플러그인이 `mock_components/GenericSystem`으로 **하드코딩**돼 있고 주석에 "won't work on real hardware".
  → 실기용은 UR처럼 **플러그인만 `fairino_hardware/FairinoHardwareInterface`로 스위칭**하면 된다.

---

## 1. FR5 확정 사실

- 조인트: `j1 j2 j3 j4 j5 j6` (revolute). 링크: `base_link → shoulder_link → upperarm_link →
  forearm_link → wrist1_link → wrist2_link → wrist3_link → tool_tcp(fixed)`. 벤더 URDF도 `j1..j6` 규약 동일.
- EE 프레임: **`ee_name = wrist3_link`** (확정). 번들 URDF에 존재 + 사용자 DLS 검증 프레임과 일치.
- 관절 한계 [rad]: j1 ±3.0543, j2 [-4.6251, 1.4835], j3 ±2.8274, j4 [-4.6251, 1.4835], j5 ±3.0543, j6 ±3.0543.
- 토크 한계 [Nm]: j1–j3 = 150, j4–j6 = 28. 홈 자세(번들): `[0, -30, -120, 70, 0, 60]°`.
- 실기: IP 기본 `192.168.58.2`, TOOL/USER=0, FR 서보 주기 **8 ms(125 Hz)**.
- 단위: FR SDK/ServoJ는 **deg**, ros2_control은 **rad** (벤더 HW가 read/write에서 변환함).
- 재사용 자산: 벤더 `fairino_description`(URDF/메시) + 번들 MuJoCo xml(`fr5_p.xml`, `fr5_v.xml`) + DLS 로직(`solvers.py`).

---

## 2. 추가/생성 패키지 (UR → FR5 매핑)

| 신규 | 담는 것 | 복제 원본 |
|---|---|---|
| `cho_description/cho_description_fr5` | URDF/xacro, `fr5.ros2_control.xacro`(mock/mujoco/**fairino** 스위칭), 메시, MuJoCo xml, config | `cho_description_ur/` + 벤더 `fairino_description` |
| `cho_controller/cho_controller_fr5` | ns `cho_controller::fr5` base/joint_space_position/task_space_ik + 액션서버 + `cho_controller_fr5.xml` | `cho_controller_ur/` (거의 그대로) |
| `cho_bringup/cho_bringup_fr5` | `bringup_{real,mujoco,gz,isaac}_robot.launch.py`, `config/{real,mujoco}/controllers.yaml`, `config/real/fr5.config.yaml` | `cho_bringup_ur/` |
| `extern/frcobot_ros2` (서브모듈, 포크) | 벤더 HW(`fairino_hardware_v3_9_9` +libfairino.so 동봉) + `fairino_msgs`; **필요한 것만 빌드**(COLCON_IGNORE) | `.gitmodules`에 추가 |
| `cho_task_manager/config/robots/fr5.yaml` | 컨트롤러 역할 single-source-of-truth | `config/robots/ur5e.yaml` |

---

## 3. 핵심: 벤더 HW 인터페이스 ↔ cho 컨트롤러 호환 (옵션 A의 실제 작업)

벤더 `FairinoHardwareInterface`(v3_9_9 기준) 분석:
- export_command: `position`만. export_state: **`position`만** (velocity/effort는 소스에 주석처리/예약).
- `on_activate`: `FRRobot` 생성 → `RPC(ip)` → 200ms 대기 → 현재각 읽어 command 초기화(급발진 방지). 실패 시 ERROR.
- `on_deactivate`: `StopMotion()` + `CloseRPC()`.
- `write`(control_mode 0): rad→deg, NaN 체크 후 **`ServoJ(..., cmdT=0.008, ...)`**. (torque 모드는 예약, 미구현.)
- IP는 `#define CONTROLLER_IP_ADDRESS`로 **하드코딩**(param 아님). `ServoMoveStart/End` **호출 안 함**
  → 이 SDK/펌웨어에선 ServoJ 스트리밍에 서보 브래킷이 불필요함이 확인됨(앞선 리뷰의 "필요할 수도" 정정).
- **속도/관절 한계 클램프 없음**(NaN 체크만) → 안전 한계는 **cho 컨트롤러 쪽**에서 책임(`task_space_ik`의 `max_delta_q` 등).

### 3.1 유일한 실질 충돌: state 인터페이스 불일치
- cho `URBaseController::state_interface_configuration()`는 조인트마다 **`position`+`velocity`**를 요구
  (`cho_controller_ur/src/base_controller.cpp:25-28`).
- 벤더 HW는 실제로 **`position`만** 연결돼 있음. **전 버전 검증**(v3_8_2_11…v3_9_9, master): `read()`의
  `GetActualJointSpeedsDegree` 호출 = 0, velocity export(비주석) = 0. 즉 위치만 `read↔GetActualJointPosDegree↔state`,
  `command↔ServoJ↔write`로 완전 연결. **속도는 export 주석 + read()에 API 호출 자체가 없음** → 활성화 실패.
  (주의: export 주석만 풀면 상시 0을 내보냄. `read()`에 속도 API 호출을 **새로 추가**해야 실값.)
- libfairino 피드백 API 존재: `GetActualJointSpeedsDegree(flag, float[6])`(deg/s), `GetJointTorques(flag, float[6])`.
  `JointPos`는 `jPos[6]`(위치)뿐.

- **A2-velocity (권장) = ros2_control 관례대로 하드웨어가 position+velocity state를 노출.** cho 컨트롤러는
  UR 복제 그대로 무수정 활성화. 벤더 HW 편집 4곳(관례적 순서: URDF 선언 → HW export → read 채움 → on_init 검증):
  1. `fr5.ros2_control.xacro`: 조인트마다 `<state_interface name="velocity"/>` 선언(command는 position 유지).
  2. `export_state_interfaces`: velocity export (관례상 `info_.joints[i].state_interfaces` 순회 권장;
     최소 수정이면 주석 해제하되 **`_jnt_velocity_state.at(i)` → `_jnt_velocity_state[i]`** — C배열이라 `.at()` 컴파일 실패).
  3. `read()`: `float sp[6]; GetActualJointSpeedsDegree(1, sp);` → `_jnt_velocity_state[i] = sp[i]*M_PI/180`. **(신규 추가)**
  4. `on_init`: state 검증을 선언된 인터페이스에 맞게(`!=1` 하드체크 → position+velocity 2개 허용).
  → `joint_state_broadcaster`가 관례대로 pos+vel 퍼블리시. effort는 별개(§3.3).

- **A1 (대안, 비추천): cho_controller_fr5를 position-only로 적응** → `state_interface_configuration()`에서 velocity 제거.
  벤더 무수정이나 **ros2_control 관례 이탈** + UR 복제 divergence + 측정속도 품질 저하.

### 3.3 effort는 "구조만" 있고 미배선 (참고)
벤더 `export_state_interfaces`/`read`의 effort 주석은 `state_data.jt_cur_tor[i]`를 참조하지만 `state_data`는
`JointPos`(torque 필드 없음) → **그대로 풀면 컴파일 실패**. 실제 토크는 `GetJointTorques`로 별도 배선 필요.
FR5 position 제어엔 불필요하므로 토크 제어기 도입 시에만 착수.

### 3.2 그 외 실기 통합 항목
- **IP 파라미터화 (확정: cho 관례 = YAML→launch→xacro param)**: `fr5.config.yaml`의 `robot_ip` → launch arg →
  xacro `<param name="robot_ip">${robot_ip}</param>` → **포크에서 `on_init`이 `info_.hardware_parameters["robot_ip"]`를
  읽어 `_controller_ip`에 대입**(v3_9_9는 `#define` 하드코딩이라 미수정 시 IP 무시됨). UR `robot_ip` 흐름과 동일.
- **update_rate = 125**: `write()`의 ServoJ cmdT가 8ms 고정 → controller_manager `update_rate`도 125로 맞춰
  스트림 주기와 일치. (불일치 시 궤적 시간/서보 버퍼 어긋남.)
- **libfairino 빌드**: `fairino_hardware`가 `libfairino`에 링크 → 해당 .so/헤더 배포 확인(colcon 빌드 경로).

---

## 4. 단계별 계획 (체크박스)

### Phase 0 — 스캐폴딩 & description
- [ ] `extern/frcobot_ros2` 서브모듈(포크) 등록(`.gitmodules`) → `fairino_hardware_v3_9_9` + `fairino_msgs`만 빌드
      (나머지 `COLCON_IGNORE`, §8.A). `libfairino` 링크 성공까지. (로봇 펌웨어는 사용자가 최신으로 업데이트.)
- [ ] `cho_description_fr5`, `cho_controller_fr5`, `cho_bringup_fr5`를 UR에서 복제·리네임
      (`_ur`→`_fr5`, ns `ur`→`fr5`, 기본 조인트명 `j1..j6`, 플러그인 등록명, package.xml/CMake).
- [ ] `fr5.ros2_control.xacro`: 하드웨어 스위칭(`mock_components/GenericSystem` / MuJoCo /
      **`fairino_hardware/FairinoHardwareInterface`**). 벤더 xacro/URDF·메시 이식 또는 참조.
- [ ] `cho_task_manager/config/robots/fr5.yaml` 추가.
- [ ] 빌드 클린: `cbp cho_description_fr5 cho_controller_fr5 cho_bringup_fr5 fairino_hardware`.
- [ ] **FK 정합성 검증**: cho URDF FK ↔ 로봇 `GetActualTCPPose`/`GetForwardKin`(mm/deg). ← 번들 리뷰 7번.

### Phase 1 — 시뮬레이션 브링업 (컨트롤러 검증)
- [ ] `bringup_mujoco_robot.launch.py` + `config/mujoco/controllers.yaml`로
      `joint_space_position_controller` / `task_space_ik_controller` 구동.
- [ ] `lambda`, `max_delta_q`, `default_dof_pos`(홈자세 rad), 속도/관절 한계 FR5화.
- [ ] `controller_check` 계열로 소진폭 point-to-point + Cartesian 추적 확인.

### Phase 2 — 벤더 HW 인터페이스 통합 (§3)
- [ ] **state 불일치 해소**: A2-velocity 권장 — 벤더 HW에 velocity export 4곳 터치(§3.1). (A1은 대안.)
- [ ] IP 파라미터화 처리(§3.2). `fr5.ros2_control.xacro`에 fairino 플러그인 브랜치 연결.
- [ ] `cho_bringup_fr5/config/real/controllers.yaml`에서 `update_rate: 125`.
- [ ] mock(`mock_components`)로 라이프사이클/컨트롤러 스위칭 스모크 → 인터페이스 계약 확인.

### Phase 3 — 실기 브링업 & 검증 (안전 최우선)
- [ ] `config/real/fr5.config.yaml`(IP/tool/user) + `bringup_real_robot.launch.py`.
- [ ] 점진 검증: ① `joint_state_broadcaster`만(상태 읽기) ② `joint_trajectory_controller` 저속 P2P
      ③ `task_space_ik_controller` 소진폭 Cartesian. cho 쪽 속도/Δq 한계 보수적.
- [ ] 비상정지 접근, 주변 클리어. `on_deactivate`의 `StopMotion` 정상정지 확인.

### Phase 4 — 문서/CI
- [ ] `CLAUDE.md` Launch에 FR5 명령 추가. `SESSION_LOG.md §0` 갱신. CI에 신규 패키지 추가.

---

## 5. 위험 & 체크포인트
- **state 인터페이스 계약**(§3.1): 미해소 시 컨트롤러 활성화 실패. Phase 2의 첫 관문.
- **URDF↔실기 프레임**: FK 대조를 Phase 0 게이트로(번들 리뷰 7번).
- **update_rate ≠ ServoJ cmdT(8ms)**: 궤적 시간/서보 버퍼 어긋남 → 125Hz로 정렬.
- **안전 한계 위치**: 벤더 write는 클램프 안 함 → cho 컨트롤러가 속도/Δq 한계 책임.
- **버전 정합**: 최신 `v3_9_9` 사용, 로봇 펌웨어도 최신으로 업데이트(사용자) → 정합 확보.
- **IP 하드코딩**: 실기 전 반드시 처리(§3.2).

## 6. 검증 지표
- 시뮬(P1): 추적 RMS/peak, 종료 수렴오차, 스위칭 스모크.
- 실기(P3): 저속 P2P 성공, 소진폭 Cartesian 추적오차, 정지 정상.

## 7. 결정사항 (모두 확정)
- [x] **state 처리 = A2-velocity** — ros2_control 관례대로 HW가 position+velocity state 노출. (effort는 §3.3, 보류.)
- [x] **`ee_name` = `wrist3_link`**.
- [x] **IP = cho 관례**(YAML→launch→xacro param) + 포크에서 `on_init`이 `hardware_parameters["robot_ip"]` 읽도록 패치.
- [x] **벤더 관리 = `frcobot_ros2` 서브모듈(포크) + 필요한 것만 빌드**(COLCON_IGNORE). 패치는 포크에 반영.
- [x] **버전 = 최신 v3_9_9** (사용자가 로봇 펌웨어 최신 업데이트).
- [x] **문서 = `todo/FR5_TODO.md` 유지**.

---

## 8. 파일 수준 체크리스트 (Phase 0–2, 확정 결정 반영)

> 전제: `ee_name=wrist3_link`, HW=`fairino_hardware_v3_9_9`(**frcobot_ros2 서브모듈, 필요한 것만 빌드·패치**),
> IP=cho 관례, A2-velocity, 조인트 `j1..j6`. description은 **번들 `fr5.urdf`/메시 기반**(j1..j6 + wrist3_link 일치;
> 벤더 `fairino_description/urdf/fairino5_v6.urdf`는 치수 교차확인용).

### 8.A frcobot_ros2 서브모듈 + 필요한 것만 빌드
> `fairino_hardware` **폴더는 버전노드**(무관). 진짜 HW는 `fairino_hardware_v3_9_9`. repo 내부 의존은
> `v3_9_9 → fairino_msgs` 하나(나머지 rclcpp/hardware_interface/pluginlib/rclcpp_lifecycle/std_msgs는 rosdep).
> libfairino.so는 v3_9_9에 동봉.
- [ ] A2-velocity + IP 패치 트래킹 위해 `FAIR-INNOVATION/frcobot_ros2` **포크** → `.gitmodules`에
      `extern/frcobot_ros2`(포크 URL) 추가.
- [ ] **빌드 대상 축소**: `fairino_hardware_v3_9_9` + `fairino_msgs`만 빌드. 나머지 `fairino_hardware_v3_9_*`·
      moveit config 폴더에 **`COLCON_IGNORE`** — 여러 버전이 동일 라이브러리명 `fairino_hardware`를 중복 export하면
      충돌하므로 필수. (`fairino_description`은 교차확인용, 빌드 제외.)
- [ ] `libfairino` .so/헤더(동봉) colcon 링크 성공 확인.

### 8.B 벤더 HW 패치 (서브모듈 포크 내부) — `fairino_hardware_v3_9_9/src/fairino_hardware_interface.cpp`
- [ ] `on_init`: state 검사 `size()!=1` → **`!=2`** + `state_interfaces[1].name==HW_IF_VELOCITY` 확인.
- [ ] `on_init`: `if (info_.hardware_parameters.count("robot_ip")) _controller_ip = info_.hardware_parameters.at("robot_ip");`
- [ ] `export_state_interfaces`: velocity export 추가(주석 해제 시 **`_jnt_velocity_state.at(i)`→`[i]`**; 권장은
      `info_.joints[i].state_interfaces` 순회).
- [ ] `read()`: `float sp[6]; if(_ptr_robot->GetActualJointSpeedsDegree(1,sp)==0) for i: _jnt_velocity_state[i]=sp[i]*M_PI/180.0;`
- (command=position 유지, effort 미착수 §3.3.)

### 8.C `cho_description_fr5` (복제원본 `cho_description_ur`)
- [ ] `package.xml`/`CMakeLists.txt` 리네임.
- [ ] `urdf/fr5.urdf.xacro`: 팔 매크로(번들 `fr5.urdf` 기반) + `fr5.ros2_control.xacro` include, `robot_ip` 인자 전달.
- [ ] `urdf/fr5.ros2_control.xacro`: 하드웨어 스위칭 — mock(`mock_components/GenericSystem`) / mujoco /
      real(`fairino_hardware/FairinoHardwareInterface` + `<param name="robot_ip">${robot_ip}</param>`).
      조인트 j1..j6: `command position`, `state position` + **`state velocity`**.
- [ ] `meshes/` 번들 STL 이식. `config/initial_positions.yaml`(홈 `[0,-30,-120,70,0,60]°`→rad).
- [ ] `xml/` 번들 MuJoCo(`fr5_p.xml`) 이식(시뮬용). `launch/view_fr5.launch.py`(선택).

### 8.D `cho_controller_fr5` (복제원본 `cho_controller_ur`, 거의 그대로)
- [ ] `package.xml`/`CMakeLists.txt` 리네임, ns `ur`→`fr5`.
- [ ] `base_controller.{hpp,cpp}`: 기본 조인트명 `j1..j6`, 기본 ee_name `wrist3_link`.
      **state config는 position+velocity 유지**(HW가 A2로 제공하므로 무수정).
- [ ] `joint_space_position_controller` / `task_space_ik_controller` / `servers/*` 이식. (gripper 제외.)
- [ ] `cho_controller_fr5.xml`: `cho_controller_fr5/JointSpacePositionController`, `.../TaskSpaceIKController` 등록.

### 8.E `cho_bringup_fr5` (복제원본 `cho_bringup_ur`)
- [ ] `config/real/controllers.yaml`: `controller_manager.update_rate: 125`; `joint_state_broadcaster`;
      `joint_trajectory_controller`(j1..j6, cmd position, state position+velocity);
      `joint_space_position_controller`/`task_space_ik_controller`(robot_type fr5, ee_name wrist3_link, j1..j6,
      lambda≈0.02, max_delta_q≈0.005, default_dof_pos=홈 rad).
- [ ] `config/real/fr5.config.yaml`: `robot_ip: 192.168.58.2`, tool 0, user 0, ee_name wrist3_link, controller_name.
- [ ] `launch/bringup_real_robot.launch.py`: **UR과 달리 벤더 control.launch include 안 함**(fairino는 cho용 launch 없음).
      직접 구성 — robot_description(xacro+robot_ip 주입) → `robot_state_publisher` → `ros2_control_node`(controllers.yaml)
      → spawner(joint_state_broadcaster, joint_trajectory_controller, +cho controller). robot_ip: fr5.config.yaml→arg→xacro.
- [ ] `config/mujoco/controllers.yaml` + `launch/bringup_mujoco_robot.launch.py`(복제원본 ur mujoco): 시뮬 검증용.

### 8.F `cho_task_manager`
- [ ] `config/robots/fr5.yaml`: robot_type fr5, controllers{joint_space: joint_space_position_controller,
      task_space: task_space_ik_controller, gripper: null, vla: null}.
- [ ] `tasks/__init__.py` 디스패치에 fr5 등록(필요 시), fr5 트리(선택).

### 8.G 빌드 & 검증 순서
- [ ] `cbp fairino_msgs fairino_hardware_v3_9_9 cho_description_fr5 cho_controller_fr5 cho_bringup_fr5` 클린.
- [ ] Phase 0 게이트: **FK 검증**(cho URDF FK ↔ `GetActualTCPPose`/`GetForwardKin`).
- [ ] Phase 1: mock/mujoco 스모크(스위칭·활성화·소진폭 추적). → Phase 3: 실기 점진(§4).
