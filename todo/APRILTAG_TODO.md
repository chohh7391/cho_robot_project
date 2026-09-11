# AprilTag 물체 위치 인식 — RealSense D435

> 목적: 태스크 트리가 **런타임에 검출된 물체 위치로 움직일 수 있게** 한다. 산출물은
> "AprilTag 노드가 뜬다"가 아니라 **`fr3_link0` 기준 `PoseStamped` 한 개**다. 거기까지 가면
> 기존 `PoseTargetBehavior` → `TaskSpaceActionBehavior(target_pose_key=...)` 경로가 그대로 받는다.
>
> 하드웨어: Intel RealSense D435. 로봇은 FR3 + Franka Hand + Bota FT.
> **장착 방식(손목 브래킷 / 삼각대)과 대수는 미정이며, 정하지 않은 채로 진행한다.**
> 그게 가능한 이유가 §1의 경계다.
>
> 2026-09-11 작성. **같은 날 Phase A(장착 위치와 무관한 부분) 구현 + 실기 검증 완료** — §5 참조.
> 남은 것은 카메라를 어디에 놓을지 정해야만 할 수 있는 일들뿐이다(§6).

---

## 0. 이 저장소에서 확인한 사실 (재확인 불필요)

### 소비자 쪽은 이미 있다 — 생산자만 없다

- [pose_target.py](../cho_task_manager/cho_task_manager/behaviors/topic/pose_target.py) `PoseTargetBehavior`:
  토픽의 `PoseStamped`를 blackboard에 latch. **repo 전체에서 이걸 publish하는 노드가 없다**
  (`grep -rn PoseTargetBehavior` → 테스트뿐). AprilTag가 첫 생산자가 된다.
- 동작 규약 셋: ① 프레임을 **변환하지 않는다**(`required_frame` 불일치 시 FAILURE), ②
  `initialise()`에서 캐시를 버리고 **그 다음 첫 메시지**를 잡는다, ③ 기본 QoS는 RELIABLE
  (`best_effort=True`로 sensor-data 퍼블리셔와 매칭 가능), 기본 타임아웃 5 s.
- blackboard 규약은 [utils/blackboard.py](../cho_task_manager/cho_task_manager/utils/blackboard.py):
  네임스페이스 `/task`, 미기록 키는 `read_if_set()`로만 읽는다(그냥 `getattr`은 KeyError가 노드를 죽인다).

### 프레임

- **절대 task-space 목표의 해석 프레임은 `fr3_link0`이다.**
  [ee_state_broadcaster.cpp:31](../cho_controller/cho_controller_franka/src/ee_state_broadcaster.cpp:31)이
  `frame_id = "fr3_link0"`로 publish한다.
- **`cho_robot_config`의 `model.base_frame: world`를 쓰면 안 된다.** 생성된 URDF
  ([fr3_franka_hand.urdf](../cho_description/cho_description_franka/urdf/fr3/fr3_franka_hand.urdf))의
  루트는 `base`이고 `base → fr3_link0`가 fixed다. **`world` 링크는 URDF에 없다** — 그 값은
  MoveIt launch가 `world_frame`으로만 쓰는 값이다. tf2 target frame은
  [franka.yaml](../cho_robot_config/config/franka.yaml)의 `model.arm_base_link`를 읽어야 한다.
- `fr3_hand_tcp`는 [end_effectors/common/franka_hand.xacro:37](../cho_description/cho_description_franka/end_effectors/common/franka_hand.xacro:37)에서 정의된다.

### 의존 패키지 현황 (2026-09-11, 이 머신 apt 기준)

| 패키지 | 상태 |
|---|---|
| `ros-humble-apriltag-ros` 3.4.0 | **설치됨** |
| `ros-humble-apriltag-msgs` 2.0.2 | **설치됨** |
| `ros-humble-realsense2-camera` 4.58.3 | 미설치 (후보 있음) |
| `ros-humble-realsense2-description` 4.58.3 | 미설치 (후보 있음) |
| `ros-humble-image-proc` 3.0.9 | 미설치 (후보 있음) |
| `ros-humble-camera-calibration` 3.0.9 | 미설치 (후보 있음) |

설치된 `apriltag_ros`는 christianrauch 계열이고 파라미터 스키마는
`/opt/ros/humble/share/apriltag_ros/cfg/tags_36h11.yaml`에 그대로 있다:
`family / size / max_hamming / qos_profile / detector.{threads,decimate,blur,refine,sharpening} /
pose_estimation_method / tag.{ids,frames,sizes}`. **`qos_profile` 파라미터가 있다** —
RealSense 퍼블리셔와의 QoS 매칭은 이걸로 해결한다.

### 기존 센서/디스크립션 패턴

- `cho_sensor/bota_ft_sensor`: extern 드라이버 위의 **config/launch/urdf 래퍼**, 노드 코드 없음.
- `cho_sensor/hansung_scale`: **cho_* 의존 0인 독립 드라이버**(CLAUDE.md가 엮지 말라고 못박음).
- 손목 액세서리는 [franka_robot.xacro:55](../cho_description/cho_description_franka/robots/common/franka_robot.xacro:55)의
  `special_connection` 플래그가 `xacro:unless`로 게이팅한다 → 카메라도 같은 방식.
- 브링업별 디스크립션 소비 방식이 **다르다**:
  real은 `special_connection: 'ft_sensor'`를 xacro에 하드코딩
  ([bringup_real_robot.launch.py:114](../cho_bringup/cho_bringup_franka/launch/bringup_real_robot.launch.py:114)),
  gazebo는 `load_ft_sensor` launch arg, **mujoco/isaac은 사전 생성된
  `urdf/fr3_with_ft_sensor/fr3_franka_hand.urdf` 파일을 읽는다.** → 카메라 링크를 추가하면
  `scripts/create_urdf.{py,sh}`로 **URDF를 재생성**해야 하고 `test/urdf_tests.py`가 따라온다.

---

## 1. 구조 — 두 층으로 나누고, 장착 의존성은 TF 한 줄로 격리한다

```
[realsense2_camera]  D435
      ↓ /camera/infra1/image_rect_raw + camera_info   (§3에서 infra1을 고르는 이유)
[apriltag_ros]  apriltag_node
      ↓ /detections (apriltag_msgs/AprilTagDetectionArray) + TF: camera_optical → tag_<id>
      ↓            ↑ camera→robot TF: 손목이면 URDF, 삼각대면 static publisher
      ↓              ← 파이프라인에서 장착에 의존하는 부분은 이 한 줄이 전부다
[cho_object_pose]  tf2 변환 + 품질 게이트 + T_tag→grasp
      ↓ /perception/object_pose  (PoseStamped, frame_id = fr3_link0, RELIABLE, 10~30 Hz)
[PoseTargetBehavior] → /task/<key> → TaskSpaceActionBehavior(target_pose_key=...)
```

가운데 두 층은 **upstream 패키지라 launch + config만 쓴다.** 직접 짜는 노드는 `cho_object_pose` 하나.

**장착 위치를 미룰 수 있는 이유:** `cho_object_pose`는 카메라 프레임 이름을 **한 번도 언급하지
않는다.** `base ← tag_<id>`만 TF에 묻고 그 사이 체인은 TF가 알아서 합성한다. 손목이든 삼각대든
바뀌는 것은 위 그림의 화살표 한 줄(그 체인의 출처)뿐이고, 노드·설정·테스트는 동일하다.

### 패키지 배치

구현된 그대로는 §5. 요점은 두 패키지로 갈랐다는 것:
`cho_sensor/realsense_apriltag`(cho_* 의존 0, 로봇을 모름) 와
`cho_perception/cho_object_pose`(로봇을 앎).

**왜 두 번째를 `cho_sensor`에 넣지 않는가:** 이 노드는 base frame 이름, grasp 오프셋,
`cho_robot_config` 레지스트리를 알아야 한다. CLAUDE.md가 `hansung_scale`에 대해 선언한
"cho_* 와 엮지 않는다" 경계를 넘는다. 대신 `cho_robot_config`에서 `arm_base_link`를 읽으면
task manager와 **같은 레지스트리**를 보게 되어 생산자/소비자 프레임이 어긋날 수 없다.

**메시지는 새로 만들지 않는다.** v1 출력은 `geometry_msgs/PoseStamped`
— `PoseTargetBehavior`가 원하는 그대로다. `cho_interfaces` 변경 없음.

### 발행 규약 (이걸 어기면 latch가 깨진다)

- `PoseTargetBehavior`는 **첫 메시지**를 잡는다 → 품질 게이트를 통과한 포즈만 publish한다.
  "일단 내보내고 나중에 고친다"가 불가능한 구조다.
- QoS는 **RELIABLE, depth 1, 연속 10~30 Hz**. TRANSIENT_LOCAL은 쓰지 않는다(구독은 `setup()`에서
  한 번 생기므로 latched 메시지가 `initialise()`의 캐시 클리어에 먹혀 어차피 무의미하다).
- `frame_id`는 반드시 `arm_base_link`. 트리 쪽은 `required_frame='fr3_link0'`.

---

## 2. 카메라를 URDF에 넣는다 — **손목 장착을 선택한 경우에만** (Phase B)

eye-in-hand는 **카메라 TF가 로봇 자세에 따라 매 순간 바뀐다.** static_transform_publisher로는
안 되고, 링크가 URDF에 있어야 robot_state_publisher가 공짜로 내보낸다.
삼각대를 고르면 이 절 전체가 `static_transform_publisher` 한 줄로 대체된다.

- `franka_robot.xacro`에 `camera:=''` 파라미터 추가, `xacro:unless value="${camera == ''}"`로
  Bota 블록과 같은 방식으로 게이팅.
- 부모 링크는 **`fr3_hand`** (브래킷이 핸드 몸체에 물림). `link8`은 이미 Bota FT가 차지했고
  `hand_tcp`는 기하가 없는 가상 프레임이다. → 실제 브래킷 CAD로 확정할 것.
- 카메라 기하는 `realsense2_description`의 `_d435.urdf.xacro`를 include (deb 설치 필요).
  광학 프레임 규약(`camera_color_optical_frame`, `camera_infra1_optical_frame`, z-forward)을
  직접 다시 쓰지 말 것 — apriltag_ros가 그 규약을 전제한다.
- **hand-eye 캘리브 결과는 xacro origin에 하드코딩하지 말고
  `config/handeye_d435.yaml`로 빼서 xacro가 읽게 한다.** 이 디스크립션이 이미
  `kinematics.yaml` / `joint_limits.yaml` / `inertials.yaml`을 그렇게 읽고 있다.

### 브링업 배선

| 브링업 | 할 일 |
|---|---|
| real | `camera:=d435` launch arg 추가 → xacro mapping. **센서 launch는 자동 include하지 않는다**(별도 실행, 초기 blast radius 최소화) |
| gazebo | `load_ft_sensor`와 같은 패턴으로 arg 추가 |
| mujoco / isaac | 사전 생성 URDF를 읽으므로 **`create_urdf.sh` 재생성 + `urdf/fr3_with_camera*/` 추가**. 카메라 이미지는 §4⑦ 참조 |

---

## 3. 캘리브레이션 — 정확도는 여기서 결정된다

### ① 스트림 선택: color가 아니라 **infra1**

D435의 **컬러 센서는 롤링 셔터, IR(infra1/2)은 글로벌 셔터**다. 손목 카메라는 팔과 함께 움직이므로
컬러로 찍으면 모션 스큐가 그대로 자세 오차가 된다. AprilTag는 어차피 그레이스케일로 변환해 쓰므로
**infra1(848×480, 글로벌 셔터)이 더 낫다.**

단, **IR 도트 프로젝터를 꺼야 한다.** 켜져 있으면 투사된 점 패턴이 태그 위에 찍혀 디코딩이 망가진다.
`depth_module.emitter_enabled: 0` — 다만 **이 장비에는 그 옵션이 없다**(§5, FW가 프로젝터를
이미 끈 상태). 깊이 스트림은 이 파이프라인에서 쓰지 않는다(AprilTag는 단안 PnP다).
추가로 **태그는 레이저 프린터로** 뽑아야 한다 — 일부 잉크젯 블랙은 근적외선 투과라 눈에는
멀쩡한 태그가 infra1에서 보이지 않는다.

### ② intrinsic

`camera_calibration` 체커보드로 직접 뽑는다. 공장 기본 camera_info를 쓰면 거리가 통째로 편향된다.
infra1을 쓰기로 했으므로 **infra1에 대해** 캘리브해야 한다(컬러 intrinsic과 다르다).

### ③ 태그 물리 규격

- family `36h11`(기본값 유지). 크기는 손목 작업거리 0.2~0.5 m 기준 **한 변 30~50 mm**.
- `size` 파라미터는 **검은 사각형 한 변의 실측값**이다. 1% 틀리면 거리가 1% 틀린다(선형).
  프린터 스케일링 때문에 **인쇄 후 캘리퍼로 재서** 넣을 것.
- 무광 용지 + 평평한 강체에 부착. 종이가 휘면 그대로 자세 오차다.

### ④ hand-eye — 바깥에서 줘야 하는 변환은 **`… → camera_link` 하나뿐이다**

드라이버가 `publish_tf: true`로 카메라 **내부** 체인(`camera_link` → 각 optical frame)을 전부
publish한다(실측 확인). 따라서 우리가 채울 칸은 `camera_link`로 들어가는 변환 하나다 —
손목이면 URDF 링크, 삼각대면 `static_transform_publisher` 하나.
`realsense2_description`으로 optical frame까지 URDF에 넣으면 **같은 static 변환의 퍼블리셔가
둘**이 되므로 하지 말 것.

손목(eye-in-hand)이면 `easy_handeye2`(humble 지원 여부 확인 필요)나 MoveIt hand-eye plugin으로
여러 자세에서 AX=XB. 삼각대면 고정 태그 하나를 base 기준 알려진 위치에 놓고 1회면 된다.

---

## 4. 함정 (이 저장소/이 하드웨어 고유)

**① `model.base_frame: world`는 TF 트리에 없다.** §0 참조. `arm_base_link`를 쓸 것.

**② 프레임 변환은 detector 쪽에서 끝내야 한다.** `PoseTargetBehavior`는 변환하지 않는다.
카메라 프레임 포즈를 흘리면 base 좌표로 오인해 엉뚱한 곳으로 간다.

**③ 실기 중력보상은 Desk의 EE 부하 설정에 의존한다 — 손목 장착 시에만.**
[base_controller.cpp:341](../cho_controller/cho_controller_franka/src/base_controller.cpp:341) —
real 경로는 `libfranka가 arm + hand 중력을 보상`하는 전제로 Coriolis만 더한다. 그 보상은
**Desk에 등록된 EE 하중 기준**이다. D435(본체 약 72 g) + 브래킷 + 케이블을 EE 부하에 반영하지 않으면
토크 컨트롤러 전체가 처진다. **카메라 장착 후 Desk 하중 재설정은 선택이 아니라 필수다.**

**④ USB 케이블이 Bota FT 센서를 가로지르면 안 된다 — 손목 장착 시에만.** FT는 `link8 ↔ hand` 사이에 있다. 케이블이
그 두 링크를 잇도록 배선되면 **자세에 따라 변하는 힘 경로**가 생겨 렌치에 편향이 들어간다.
tare는 한 자세에서만 유효해지고, forge 태스크의 접촉 임계값과
[safety_monitor.py](../cho_task_manager/cho_task_manager/behaviors/topic/safety_monitor.py)의
FT 가드가 같이 흔들린다. 케이블은 FT 아래(link8 쪽)에서 팔을 따라 고정할 것.

**⑤ TF lookup은 이미지 타임스탬프로.** (손목 장착이면 필수, 삼각대여도 물체가 움직이면 같다.)

`lookup_transform(..., msg.header.stamp)`. `now()`로 하면 이미지 지연만큼 어긋난다.
**v1은 "정지 → 인식 → 이동" 시퀀스로 짜서 이 문제를 회피한다.**

**⑥ AprilTag 자세 ambiguity.** 평면 태그는 정면·소형일수록 회전 해가 둘로 갈려 z축이 플립된다.
**위치는 믿을 만하지만 자세는 못 믿는다.** v1은 태그에서 **위치 + yaw만** 쓰고 접근 자세는
top-down 고정. 자세가 꼭 필요하면 태그 번들로 간다.

**⑦ 시뮬레이션으로는 검출부를 검증할 수 없다.** `mujoco_ros2_control`은 카메라 이미지를 ROS로
내보내지 않는다. 따라서 **(a) 검출부는 책상 위 실물 카메라 + 인쇄 태그로**, **(b) 트리 배선은
`PoseStamped`를 쏘는 mock 노드로** 나눠서 검증한다. Isaac 카메라 퍼블리시는 v1 범위 밖.

**⑧ 움직이는 팔 위의 USB3 — 손목 장착 시에만.** 케이블 스트레인으로 D435가 끊기면 `dmesg`에 USB reset이 뜬다.
짧은 직각 USB-C + 팔을 따라 스트레인 릴리프. 노드는 스트림 끊김을 **에러로 승격**해야 한다
(조용히 멈추면 트리는 5 s 타임아웃으로만 알게 된다).

**⑨ `-Ofast`.** v1은 Python이라 무관하지만, 나중에 C++로 옮기더라도 **finiteness 검사를
`cho_controller_common`에 넣지 말 것**(CLAUDE.md).

---

## 5. Phase A — 장착 위치와 무관한 부분 (2026-09-11 완료)

`colcon test` 43개 통과. `.github/workflows/ci.yml`의 BUILD/TEST 목록에 두 패키지 등록.

### `cho_sensor/realsense_apriltag` (ament_cmake, 설정 + launch만)

| 파일 | 내용 |
|---|---|
| `config/d435.yaml` | upstream `rs_launch.py`의 `config_file`로 넘기는 **평탄한** 매핑. infra1 480x270x30, `publish_tf: true`(드라이버가 카메라 내부 체인을 소유), depth/infra2 off. 모든 값 실측 확인 |
| `config/apriltag_36h11.yaml` | `decimate: 1.0`(상용 기본 2.0은 작은 태그에서 손해), `max_hamming: 0`, `qos_profile: sensor_data` |
| `launch/apriltag.launch.py` | upstream `rs_launch.py` include + (color일 때만) image_proc + apriltag_node. `stream:=infra1\|color`, `launch_camera:=false`(bag 재생용), `frame_prefix:=`, `camera_config:=` |

`tag.frames`는 설정 파일에 쓰지 않는다. launch가 `tag.ids`에서 `<prefix>tag_<id>`로 생성하고,
소비자 쪽 `geometry.tag_frame_name()`이 같은 규약을 적용한다 — 손으로 쓴 프레임 문자열이
양쪽에 존재하지 않으므로 어긋날 수가 없다.

### `cho_perception/cho_object_pose` (ament_python)

| 파일 | 내용 |
|---|---|
| `geometry.py` | 쿼터니언 헬퍼, 디코드 게이트, 집계, yaw-only 투영. **ROS 없음 / 시계 없음 / 카메라 없음** |
| `objects.py` | `config/objects.yaml` 파싱·검증. 같은 태그 id나 같은 토픽에 두 물체 → 에러 |
| `node.py` | `/detections` 구독 + **이미지 타임스탬프로** tf2 조회 + 게이트 + 발행 |
| `mock_publisher.py` | 고정 포즈 발행. 카메라 없이 태스크 트리를 먼저 배선·검증하기 위한 것 |

`cho_vla_core`와 같은 분할이고 이유도 같다 — 검증할 가치가 있는 부분은 로봇이 필요 없다.

**설계 판단 넷 (계획에 없던 것)**

- **노드가 카메라 프레임 이름을 아예 모른다.** `base ← tag_<id>`만 TF에 묻고 중간 체인은 TF가
  합성한다. 이게 장착 위치를 미루고도 여기까지 올 수 있었던 이유 전부다(§1).
- **자세는 medoid, 위치는 median.** 평균이 아니다. AprilTag의 평면 자세 2해 모호성 때문에 플립을
  가로질러 평균내면 **관측된 적 없는** 회전이 나오고 두 해 어느 쪽과도 가깝지 않다. medoid는 항상
  실제로 본 것을 돌려준다.
- **게이트는 발행 *전에* 끝난다.** `PoseTargetBehavior`가 첫 메시지를 latch하므로 나쁜 포즈가
  토픽에 닿은 시점에 이미 따라간 뒤다. 점수를 붙여 내보내는 선택지가 없다.
- **`frame_prefix` (질문에서 나온 다대수 대비).** 두 detector를 기본값으로 띄우면 **둘 다 `tag_9`를
  publish**하고, TF 트리에 부모가 둘인 자식이 생긴다. 에러로 뜨지 않고 늦게 도착한 쪽으로
  간헐적으로 해석되는 형태로 망가진다. 카메라마다 prefix를 주면 갈라진다. 지금은 물체 하나가
  카메라 하나에 속하는 경우까지 지원한다(N대 동시 융합은 §6).

### 드라이버는 감싸기만 한다

`realsense2_camera`를 직접 노드로 띄우지 않고 **upstream `rs_launch.py`를 include**하고,
그쪽의 `config_file` 인자로 우리 yaml만 넘긴다(`bota_ft_sensor`와 같은 배치). 두 가지 주의:

- `config_file`은 `yaml.safe_load` 결과를 그대로 노드 파라미터로 넘긴다 → **평탄한 매핑**이어야
  한다. 평소의 `/**: ros__parameters:` 형태로 쓰면 `'/**'`라는 이름의 파라미터 하나가 된다.
- 노드는 `[launch 인자, config 파일]` 순으로 받고 **뒤가 이긴다.** 그래서 `enable_color` /
  `enable_infra1`은 yaml이 아니라 launch 인자여야 `stream:=`로 바꿀 수 있다.
- include를 `GroupAction(scoped=True, forwarding=False)`로 감쌌다. rs_launch가 컨텍스트의
  모든 launch configuration을 자기 파라미터 목록과 대조해 경고를 찍는데, 감싸지 않으면 우리
  인자(`stream`, `frame_prefix` …)까지 흘러들어가 **노란 경고 다섯 화면**이 나온다. 진짜 경고가
  묻히는 전형적인 방식이라 막았다.

### 실기 검증 (D435 s/n 844212070094, realsense2_camera 4.58.3, 2026-09-11)

| | |
|---|---|
| 검증됨 | 단위 테스트 43개 + lint. **USB 3 / 848x480x30**에서 `infra1/image_rect_raw` 29.1~30.0 Hz, `/detections` **29.996 Hz**(전체 해상도에서도 매 프레임 소화), `camera_info.frame_id = camera_infra1_optical_frame`, 드라이버 TF `camera_link → camera_infra1_optical_frame` RPY `[-1.571, 0, -1.571]`, intrinsics **`fx=fy=423.94 cx=427.43 cy=245.69`**(USB 2 폴백 480x270에서는 `fx=fy=239.96 cx=241.94 cy=138.22`) |
| **검출 실증** | 모니터에 띄운 태그로 끝까지 확인: `tag36h11 id=9 hamming=0 decision_margin=206`, TF `camera_infra1_optical_frame → tag_9 = [-0.117, 0.064, 0.349]`. 임시 `fr3_link0 → camera_link` static TF를 꽂으니 `cho_object_pose`가 `/perception/object_pose/cube`를 `fr3_link0` 기준 `[0.404, 0.091, 0.240]`으로 297건 발행. 게이트 셋 다 실제로 동작(`edge 25.0 px < 25.0` 거부 / 손으로 들었을 때 `spread 60.3 mm > 10.0` 거부 / 정지 시 `spread 1.0 mm over 15 samples` 발행) |
| **미검증** | intrinsic 캘리브, **계측 정확도**(config의 태그 크기가 실제 띄운 태그와 맞아야 거리가 의미를 가진다), color 경로 |

**rviz로 보는 법**: `rviz:=true`를 주면 `apriltag_draw`(검출을 이미지에 그려 `/image_tags`로
재발행)와 `rviz/apriltag.rviz`를 띄운다. 두 가지 함정이 있었다 — `apriltag_draw`는 **lazy
구독**이라 `/image_tags`를 누가 구독해야 프레임을 당겨오고, 검출 입력 토픽 이름이
**`tags`**지 `detections`가 아니다. 후자로 remap하면 조용히 아무 일도 안 일어나고 증상이
"태그가 검출 안 됨"과 똑같아 보인다.

**추측이 틀렸던 파라미터 셋 (전부 실측으로 교정)**

- `depth_module.infra_profile`은 `'848x480x30'` 형식이다. rs_launch의 기본값이 `'0,0,0'`이라
  콤마 구분인 줄 알고 `'848,480,30'`을 넣었는데 **런타임에 거부되고 조용히 기본값으로 되돌아간다**
  (`Given value, 848,480,30 is invalid ... Setting ROS param back to: 480x270x30`).
- `depth_module.emitter_enabled`는 **이 장비에 존재하지 않는다.** 드라이버가
  `Projector capacity is overrided and disabled by FW`를 찍고 해당 파라미터를 선언하지 않는다.
  설정에서 빼고 README에 확인 방법만 남겼다.
- `depth_module.exposure`는 **double**이다. upstream `rs_launch.py`가 자기 기본값 `8500`을
  정수로 넘겨서 노드가 거부한다 — 로그의 그 경고는 우리 것이 아니다.

### ⚠ USB 3 링크가 아직 신뢰할 수 없다

처음엔 `RealSense USB2` / `480M`로 올라와서 infra가 424x240·480x270뿐이고 RGB 센서가 안 보였다.
포트냐 케이블이냐는 sysfs가 답한다 — xHCI는 USB2/SuperSpeed 버스를 따로 노출하고, 물리 커넥터가
USB 3이면 두 포트 객체가 `peer`로 연결된다:
`/sys/bus/usb/devices/usb3/3-0:1.0/usb3-port4/peer -> ../../../usb4/4-0:1.0/usb4-port4`.
짝이 있으니 포트는 USB 3 지원 → 케이블. 두 번째 케이블은 USB 3 협상까지는 되는데,
**SuperSpeed 세션이 2분을 넘긴 적이 없다.** 서로 다른 xHCI 컨트롤러 두 곳 모두에서:

    케이블 A            USB 3 협상 자체가 안 됨; 480M에서는 안정
    케이블 B, 포트 4-4  13:48:52 up -> 13:50:26 drop  (~94초)
                        13:54:55 up -> 13:55:03 drop  (8초, 스트리밍 없음)
                        13:55:06 up -> 13:56:34 USB 2(0ad6)로 폴백
                        14:00:58 up -> 14:01:13 drop  (15초)
    케이블 B, 포트 2-1  14:01:32 up -> 14:03:31 drop  (~119초)
      (완전히 다른 xHCI 컨트롤러)

좁혀지는 것 둘:

- **부하 문제가 아니다.** 13:55:03 끊김은 열거 8초 뒤, 아무것도 스트리밍하지 않는 상태였다.
  대역폭이나 스트리밍 중 전력 문제가 아니라는 뜻.
- **포트 문제가 아니다.** 다른 컨트롤러의 포트로 옮겨도 끊기는 시점만 달라졌다.

남는 건 케이블 또는 카메라 자체의 USB-C 커넥터(D435의 알려진 마모 지점)다.

**지금 쓰는 게 C-to-C 케이블인데, 이게 첫 번째로 바꿀 것이다.** D435 동봉 케이블은
USB-C **to USB-A**이고, D400 시리즈에서 C-to-C는 오래된 문제 지점이다 — 카메라 쪽은 평범한
device 포트라서 일부 C-to-C 조합이 USB 2로 떨어지거나 SuperSpeed를 잠깐만 유지한다.
지금 증상과 정확히 일치한다. (Intel 가이드와 librealsense 이슈 트래커 근거이고 이 머신에서
측정한 것은 아니다.)

지금 포트가 더 얹는다: `2-1`은 `00:0d.0` = **Meteor Lake-P Thunderbolt 4 USB 컨트롤러**라
TBT 롤/알트모드 검출까지 링크에 끼어든다. 다른 쪽 `80:14.0`은 평범한 PCH xHCI다.

그래서 가장 정보량이 큰 다음 한 수는 두 변수를 동시에 없애는 것: **동봉 C-to-A 케이블 +
PCH 컨트롤러의 SuperSpeed USB-A 포트**(`usb3-port` 2·3·4·9·10이 짝을 가진다). 그리고 soak —
2분 안에 터지는 증상이니 `journalctl -k -f` 띄워두고 5분 스트리밍하면 결론이 난다.

이 머신엔 Type-C 포트 매니저가 없어서(`/sys/class/typec` 부재) CC/롤 협상을 OS에서 들여다볼
수 없다. 케이블 교체 자체가 측정이다.

그동안 **USB 2 폴백은 안정적이므로**(75~90초 실행 여러 번 무결함) 480x270 · 작업 거리 절반으로
커미셔닝은 진행할 수 있다.

**그래서 프로파일을 config에서 빼 launch 인자로 올렸다**(`profile:=`, 기본 `848x480x30`).
config에 두면 launch 인자를 이길 수 없어서 링크에 따라 바꿀 수가 없다. USB 2 링크에서 848을
요구하면 드라이버가 뜨기도 전에 launch가 경고한다(실측 확인). **자동으로 고르지는 않는다** —
해상도가 바뀌면 intrinsic이 바뀌고, 조용히 바꾸면 캘리브레이션과 픽셀 기준 임계값이 말없이
무효가 된다.

USB 2에서 만들고 USB 3에서 돌리는 건 된다. 넘어가지 않는 것은 둘뿐이다:

| | |
|---|---|
| 그대로 간다 | 코드·테스트 전부, 프레임 규약, `tag_<id>` 명명, `objects.yaml`과 grasp 오프셋, 태스크 트리 배선, 실측한 태그 크기, camera→robot extrinsic(마운트 변환이지 픽셀량이 아니다) |
| 최종 프로파일에서 다시 | **intrinsic 캘리브레이션**, 그리고 거기에 맞춰 튜닝한 픽셀 임계값 — 특히 `min_edge_px`. `fx`가 239.96 → 423.94로 바뀌고 작업 거리도 0.38 m → 0.68 m로 바뀐다 |

즉 지금은 `profile:=480x270x30`으로 전부 만들고 배선하되, **캘리브는 케이블이 정해진 뒤 최종
프로파일에서 한 번만** 한다.

커넥터를 흔들며 실시간 관찰: `journalctl -k -f | grep -E "usb [0-9]-[0-9]+"`

**손목 장착은 이 문제를 엄격히 악화시킨다**(§4④⑧) — 브래킷 전에 반드시 정리할 것.

USB 2가 가리고 있던 것 (기록용):

| | USB 2 | USB 3 |
|---|---|---|
| USB id | `8086:0ad6`(USB 2 일반 식별자) | `8086:0b07`(D435) |
| 드라이버 device name | `RealSense USB2` | `RealSense D435` |
| infra profile | 424x240, 480x270 | + 640x360/400/480, **848x480(90 fps까지)**, 1280x720/800 |
| color | `rgb_camera.*` 0개 | 22개, 최대 1920x1080x30 |
| emitter 옵션 | 없음 | **여전히 없음** |

**예측이 절반 틀렸다**: color는 돌아왔지만 emitter는 USB 3에서도 없다. 링크가 아니라 이 개체의
펌웨어가 프로젝터를 꺼둔 것이고(IR 검출엔 유리) 설정할 게 없다.

### 어느 카메라를 열지 고르는 법

**연결에 포트 지정은 필요 없다** — 드라이버는 처음 발견한 RealSense를 잡는다. 두 대 이상일 때
`rs_launch.py`가 제공하는 선택자는 셋이고 전부 `config/d435.yaml`에 넣을 수 있다:
`serial_no`(카메라 자신의 시리얼 — **기본 선택지**, 어느 포트에 꽂아도 따라간다),
`usb_port_id`(USB 경로 `2-1` — 위치가 의미를 갖는 고정 리그용),
`device_type`(모델명 `d435`). 시리얼은 **librealsense가 보고하는 값**을 쓸 것 —
커널 USB 디스크립터의 `SerialNumber`와 다른 필드다(여기서는 `844212070094` vs `846623021037`).
`rs-enumerate-devices -s`로 확인한다. ROS 밖에서는 `rs2::config::enable_device(serial)`
(파이썬은 `cfg.enable_device('...')`)이 같은 역할이다. 단 **파이썬 바인딩은 설치돼 있지 않다**
— `ros-humble-librealsense2`는 C++ 라이브러리와 `rs-*` CLI만 준다.

### upstream 노드가 종료 시 segfault

`apriltag_node`와 `realsense2_camera_node` 둘 다 launch 종료 때 libc 안에서 segfault 한다
(매번, `journalctl -k`). 작업이 끝난 뒤의 종료 경로이고 둘 다 우리 코드가 아니다. 이 패키지의
결함으로 오인하지 않도록 기록해 둔다.

---

## 5a. 태스크 통합 (2026-09-11)

**일반은 `cho_object_pose`, 구체는 태스크**로 갈랐다. 파이프라인·게이트·프레임 해석은 perception이
갖고, "어느 태그가 무슨 물체이고 어디로 가야 하나"는 태스크가 갖는다.

| | 소유 |
|---|---|
| tag id → 물체, grasp 오프셋, 출력 토픽 | 태스크 테이블 (`cho_task_manager/config/perception/<task>.yaml`) |
| `min_samples`, `max_position_spread_m` | 태스크 — **요구 정확도**다 |
| `max_hamming`, `min_decision_margin`, `min_edge_px` | perception — 광학·intrinsic의 함수 |
| `base_frame` | 어느 쪽도 아님. `cho_robot_config`에서 읽어 드리프트를 원천 차단 |

`run_task_manager.launch.py`에 `object_pose_config:=` 인자를 추가했고, **비어 있으면 perception을
아예 include하지 않는다** — 태그를 안 쓰는 로봇/태스크는 비용 0(실측: `task:=pick_place`로
띄우면 `object_pose_node` 0개). `cho_task_manager`의 의존은 **exec_depend 하나**고 import는 없다.

첫 소비자로 `tasks/franka/tag_reach.py`를 만들었다: home → `PoseTargetBehavior`로 검출 대기 →
`TaskSpaceActionBehavior(target_pose_key=...)`로 이동 → home. **트리 어디에도 좌표가 없다.**
테이블은 태그 면에서 100 mm 떨어진 **standoff**로 잡았다 — intrinsic 캘리브 전에는 검출 거리가
설정한 태그 크기만큼만 정확하므로, 스케일 오차가 충돌이 아니라 빗나감이 되게 한다.

부수 수정: `controller_names.load_robot_config`(태스크 매니저용 compat view)에 `arm_base_link`를
추가했다. 태스크가 프레임을 알아야 하는데 compat view에 `model`이 없어서 `KeyError`가 났다.
레지스트리를 태스크마다 다시 열거나 프레임을 문자열로 박는 것보다 낫다.

검증: `cho_task_manager` 116개 + `cho_object_pose` 35개 + `realsense_apriltag` 8개 통과.
launch 조립도 실측 — 태스크 테이블로 `target<-tag_9 -> /perception/object_pose/target` 기동 확인.

---

## 5b. MuJoCo 통합 테스트 (2026-09-11)

**시뮬 로봇 + 실제 카메라** 조합으로 돌렸다. MuJoCo는 카메라 이미지를 ROS로 안 주므로(§4⑦)
팔만 시뮬, 타깃은 실제 태그에서 온다. 하드웨어 위험 없이 전체 체인을 도는 구성이다.

```
MuJoCo (torque, task_space_qp_controller)
실제 D435 + apriltag              →  tag36h11 id=9, decision_margin 164~195
임시 마운트 static TF (fr3_link0 → camera_link)
cho_object_pose                   →  /perception/object_pose/target, 산포 0.7~3.9 mm
tag_reach 트리                     →  검출 좌표로 이동
```

**결과: 2회 SUCCESS** (run2, run4). 트리 로그가 전 과정을 남긴다:

    [Switch_To_joint_space_impedance_controller] Controllers Switched Successfully!
    [Go_Home] Action Succeeded!   [Open_Gripper] Action Succeeded!
    [Detect_Tag] target /task/tag_target_pose = [+0.37293, +0.03339, +0.16008] m in 'fr3_link0'
    [Switch_To_Task_Space] Controllers Switched Successfully!
    [Move_To_Tag_Standoff] Action Succeeded!
    [Go_Home_Final] Action Succeeded!
    Task finished with status: Status.SUCCESS

도달 여부는 **액션 서버의 수렴 판정**이다. TCP를 독립적으로 재보려 했으나 그 회차에 카메라가
죽어 실패했고, 유효한 독립 측정은 아직 없다.

**테스트가 실제 버그를 하나 잡았다.** `tag_reach`에 **컨트롤러 전환이 빠져 있었다** —
`home_subtree`가 joint impedance를 켜놓은 채 끝나는데 task-space 액션 서버는 자기 컨트롤러가
active일 때만 답한다. `ur/multi_move.py`의 패턴대로 `SwitchControllerServiceBehavior`를 검출과
이동 사이에 넣었고(검출을 기다리는 동안 팔은 계속 잡혀 있게), 컨트롤러 이름도 하드코딩 대신
`robot_config['task_space']`에서 가져온다. 순서는 테스트로 고정했다.

**시간 기준 함정.** MuJoCo 기본은 `use_sim_time:=true`라 로봇 TF가 sim clock(`sec=45`)으로
찍히는데 실제 카메라는 wall clock(`1789107427`)이다. 이미지 stamp로 TF를 조회하는 설계라
그대로면 extrapolation 에러다. **하이브리드 테스트는 `use_sim_time:=false`로 시계를 하나로**
맞춰야 한다.

**safe-abort도 검증됐다.** 카메라가 죽은 회차에서 `Detect_Tag`가 15초 타임아웃 →
`Abort_Switch_To_Hold_torque`가 hold 컨트롤러로 전환 → `Status.FAILURE`. 그냥 죽지 않고
팔을 잡아두고 끝난다.

**upstream 불안정 둘.** `realsense2_camera_node`가 **실행 중에도** segfault 한다(종료 시뿐이
아니다, `journalctl -k` 15:23:58). 그리고 USB 링크가 여전히 몇 분 단위로 떨어진다 — 5회 실행 중
2회가 이것 때문에 실패했다. 태스크 쪽 결함이 아니다.

---

## 6. Phase B — 장착을 정해야 할 수 있는 일

| | 손목 브래킷 (eye-in-hand) | 삼각대 (eye-to-hand) |
|---|---|---|
| camera→robot TF | URDF에 카메라 링크(§2) → `robot_state_publisher` | `static_transform_publisher` 하나 |
| 캘리브 | AX=XB, 팔을 여러 자세로 | 고정 태그 하나로 base 기준 1회 |
| Desk EE 하중 | **재설정 필수**(§4③) | 무관 |
| FT 케이블 경로 | **주의**(§4④) | 무관 |
| URDF 재생성 | 필요(mujoco/isaac) | 불필요 |
| 롤링 셔터 | infra1이어야 함 | color도 가능 |

즉 **삼각대 쪽이 압도적으로 싸다.** 먼저 삼각대로 파이프라인 전체를 세우고, 손목 장착은
그게 동작한 뒤에 붙이는 순서를 권한다 — 위 표에서 손목 전용 항목 넷이 전부 나중으로 밀린다.

### 카메라 위치·대수를 task로 다루는 것에 대해 (2026-09-11 논의)

**둘로 갈라야 한다.**

**task가 맞는 것 — 절차(motion + 측정의 시퀀싱).** 이건 BT가 존재하는 이유 그 자체다.

- **hand-eye 캘리브 task.** N개 자세로 이동 → 각 자세에서 태그 관측 latch → AX=XB → yaml 기록.
  선례가 이미 있다: MIT 튜닝 task가 측정값을 `/mit_tuning` blackboard 네임스페이스에 쌓는다
  (`utils/blackboard.py`). 캘리브도 같은 모양이다.
- **탐색(viewpoint) task.** `Selector(짧은 타임아웃의 PoseTarget, 다음 시점으로 이동)`.
  시점 목록은 장착과 대수에 따라 달라지지만 그건 **task의 설정**이고, 거기 있는 게 맞다.
- **어느 카메라를 쓸지 고르는 분기.** 토픽/키를 갈아끼우는 것이므로 task 레벨 선택이 자연스럽다.
- 단, 탐색 task는 §4⑤와 얽힌다 — **정지 후 인식**이어야 한다. 이동 중 latch하면 타임스탬프 정합이
  그대로 오차로 들어온다.

**task가 아닌 것 — 정적 기하와 토폴로지.**

- **extrinsic(카메라가 어디 있는지)은 TF에 있어야 한다.** task가 들고 있으면 모든 task가 같은 숫자를
  다시 적게 되고, 둘이 어긋나는 순간 아무도 어느 쪽이 맞는지 모른다 — `cho_robot_config`가
  존재하는 이유와 정확히 같은 실패 양식이다. 게다가 rviz, MoveIt 충돌 검사, 인식 노드는 전부 TF를
  읽는다. task는 그것들에게 값을 먹일 수 없다.
- **카메라 대수/인스턴스는 launch에 있어야 한다.** 노드를 몇 개 띄우느냐는 실행 구성이지 행동이 아니다.
  그리고 대수가 늘어나는 순간 `frame_prefix`가 필수가 되는데(§5), 그건 detector와 소비자 **양쪽**의
  기동 파라미터라 task가 관여할 수 있는 지점이 아니다.

요약: **"카메라가 어디에 있는가"는 TF, "카메라로 무엇을 하는가"는 task.** 전자를 task에 넣으면
드리프트하고, 후자를 launch에 넣으면 로봇을 움직일 수 없다.

---

## 7. 열려 있는 결정

- **USB 3 케이블/포트.** 지금 USB 2라 480x270에 묶여 있고 작업 거리가 약 0.38 m다(§5).
  장착 결정보다 먼저 해치울 수 있고 효과가 가장 크다.
- **장착 방식.** 위 표 기준으로는 삼각대 먼저를 권한다.
- **카메라 N대 동시 융합.** 지금은 "물체 하나 = 태그 하나 = 프레임 하나"다. 두 카메라가 같은 물체를
  보게 하려면 `ObjectSpec`이 프레임 목록을 갖고 노드가 `/detections`를 여러 개 구독해야 한다.
  집계 창은 출처를 구분하지 않으므로 샘플은 그냥 섞여 median에 들어간다 — 배관만 늘어나고
  알고리즘은 그대로다. 필요해지면 그때.
- **`cho_perception` grouping.** 지금 패키지 하나뿐이다. 그래도 `cho_sensor`의 자기완결성 원칙을
  깨지 않으려면 여기가 맞다(§1).
- **`vla` 경로와의 관계**: VLA는 자체 관측 파이프라인을 쓴다. AprilTag는 고전 태스크 트리용이고
  둘을 합치지 않는다 — 합치려면 별도 설계가 필요하다.
