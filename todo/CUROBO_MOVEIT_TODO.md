# cuRobo — MoveIt 플래너 파이프라인 통합 계획 (A안: `isaac_ros_cumotion` MoveIt 플러그인)

> 목적(당초): **플래닝 속도**. OMPL 대비 계획 시간을 줄인다.
>
> **2026-09-08 측정 결과 그 목적으로는 얻을 것이 없었다.** 계획 시간은 OMPL 이 3배 빠르다.
> 대신 측정이 다른 이득을 찾아냈다: 좁은 씬의 **task(pose) 목표 도달 성공률**
> (cuMotion 27/27 대 OMPL 23/27, 충돌 인식 IK 덕분). joint 목표는 OMPL 이 이긴다.
> 판정과 원자료는 `todo/curobo_bench/README.md`. 기본값은 OMPL 로 남긴다(D2).
>
> 방식: cuMotion을 MoveIt의 **두 번째 planning pipeline**으로 등록한다. `move_group`,
> `moveit_action_bridge`, 씬 게이트, 컨트롤러 전환 트랜잭션은 전부 그대로 두고 플래너만
> 갈아끼운다. **task(포즈) 목표 전용**, **로봇별 opt-in**, 기본값은 계속 OMPL.
>
> 되돌리기 비용이 거의 0인 것이 이 설계의 핵심이다. 판정에서 지면 파이프라인 등록만 빼면 된다.

---

## 0. 확정 사실 (2026-09-08 조사, 재확인 불필요)

### 환경

- ROS 2 **Humble** / Ubuntu 22.04 / MoveIt **2.5.9**(apt) / 시스템 python **3.10.17**
- GPU **RTX 5070 = sm_120 (Blackwell)**, 드라이버 580, CUDA 12.8·12.9·13.x 설치됨
- Humble rclpy는 `_rclpy_pybind11.cpython-310-*.so` → **cp310 전용**.
  Isaac Sim 6.0.1의 번들 python은 **3.12.13**이라 사용 불가 (§4 참조).

### 상류 패키지

- Humble을 지원하는 마지막 버전은 **`release-3.2`** (패키지 버전 3.2.5). `main`은 Isaac ROS
  4.6 / **Jazzy / py3.12 / cuRoboV2**로 넘어갔다. 즉 우리가 올라타는 것은 **동결된 가지**다.
  단 **없는 게 아니라 멈춰 있는 것**이다 — jammy 패키지는 풀세트가 실재한다.

  | 배포판 | 저장소 | 버전 | 최종 갱신 |
  |---|---|---|---|
  | jammy (22.04 / Humble) | `release-3` | 3.2.5 | 2025-11-06 |
  | noble (24.04 / Jazzy) | `release-4` | 4.6.0 | 2026-08-18 |
- apt에 존재 확인: `ros-humble-isaac-ros-cumotion-moveit` 3.2.5, `ros-humble-curobo-core` 3.2.5,
  `ros-humble-nvblox-msgs` 3.2.5(1.7 MB, 메시지 전용), `ros-humble-isaac-ros-common` 3.2.5(151 KB).
- **`curobo-core` deb는 `Depends`가 없다** — Isaac ROS 도커 안의 torch를 전제로 빌드된 물건이다.
  그 torch는 cu126 세대로 추정되어 sm_120 커널이 없을 가능성이 높다. **deb를 쓰지 않고 소스 빌드한다.**

### 플러그인 동작 (소스 확인 완료)

- 플러그인 클래스: **`isaac_ros_cumotion_moveit/CumotionPlanner`**.
  release-3.2 설정은 `planning_plugin`(단수) + `default_planner_request_adapters` → Humble MoveIt과 일치.
- 플러그인은 계획을 직접 하지 않고 **`cumotion/move_group` 액션**(`moveit_msgs/MoveGroup`)으로
  파이썬 노드에 넘긴다. 그 노드가 cuRobo `MotionGen`으로 푼다.
- `cumotion_move_group_client.cpp`가 **전체 planning scene을 `planning_scene_diff`에 실어 보낸다**
  → `publish_static_scene.py`가 올린 바닥은 정상 전달된다. ✅
- 파이썬 노드는 `scene.world.collision_objects`만 읽는다 → **attached collision object는 무시된다.**
- **joint goal은 FK로 EE 포즈를 만든 뒤 `plan_single(pose)`로 푼다** (`cumotion_planner.py` 713–783).
  요청한 관절각이 그대로 지켜지지 않는다. → §1 D1의 근거.
- Cone primitive 미지원. mesh는 지원.
- `max_velocity/acceleration_scaling_factor` → `time_dilation_factor = min(둘)`로 뭉개져 매핑된다.
- **프레임 변환을 하지 않는다.** 포즈를 `Pose.from_list`로 그대로 받는다
  → MoveIt 계획 프레임과 XRDF base가 같아야 한다.
- 문서 명시: **`move_group`은 동시에 하나만** 띄울 것 (동시 요청 시 CUDA 에러).

### XRDF

- `robot` 파라미터는 파일명을 `isaac_ros_cumotion_robot_description/xrdf`에 join하지만,
  cuRobo `join_path`는 **path2가 절대경로면 path1을 무시**한다
  → **우리 패키지 안의 XRDF를 절대경로로 넘길 수 있다. NVIDIA 패키지 포크 불필요.** ✅
- `get_robot_config()`가 `attached_object` 링크에 스피어 버퍼 100개를 붙인다
  → XRDF에 `add_frame: attached_object` modifier가 **있어야 한다**.
- 상류 제공: `ur5e.xrdf`, `ur10e*.xrdf`, `franka.xrdf`.
  **`franka.xrdf`는 조인트명이 `panda_*`라 우리 `fr3_*` 기술과 맞지 않는다** → 재사용 불가.

- **FR5 XRDF는 이미 있다** (2026-09-08 확인). `~/sdl_ws/src/sdl_project/TAMP/tamp/content/`:

  | 경로 | 내용 |
  |---|---|
  | `assets/robot/dcp_description/config/fr5/fr5.xrdf` | 팔 전용 (j1~j6) |
  | `assets/robot/dcp_description/config/fr5_ag95/fr5_ag95.xrdf` | 그리퍼 포함 (14 DOF) |
  | `assets/robot/dcp_description/urdf/robot/fr5.urdf` | **평문 URDF** — xacro 확장 없이 `urdf_path`에 바로 사용 가능 |
  | `configs/robot/spheres/*.yml` | fr5 / fr5_ag95 / fr5_dh3 / fr5_vgc10 |

  이름이 우리 기술과 **정확히 일치**한다: 조인트 `j1..j6`, 링크
  `base_link / shoulder_link / upperarm_link / forearm_link / wrist1_link / wrist2_link / wrist3_link`.
  우리 MoveIt 그룹도 `base_link → wrist3_link`다. 스피어는 링크 로컬 좌표이므로 우리 URDF로 그대로 전이된다.
  또한 우리 `fr5.urdf.xacro`는 `world → base_link`를 **identity**로 붙이므로 계획 프레임이 자동 정합된다.

  적응이 필요한 부분만 남는다: ① `attached_object` 프레임(`add_frame` modifier) 부재 —
  `get_robot_config()`가 그 링크에 스피어 100개를 무조건 붙이므로 cuRobo가 링크를 못 찾을 수 있다(3줄 추가로 해결),
  ② `tool_frames` 부재 — 노드 파라미터 `tool_frame`으로 주면 무관, ③ ag95 변형은 그리퍼 조인트/링크명이
  우리와 다르다(dcp `gripper_finger1_joint`/`knuckle`/`finger_tip` vs 우리 `gripper_finger_joint`/`gripper_base_link`)
  — 팔만 계획하므로 지금은 무관, 그리퍼 충돌까지 넣을 때 새로 쓴다.

- 속도 튜닝 지렛대: 위 XRDF의 `acceleration_limits`가 전부 10, `jerk_limits`가 전부 10000인
  균일 기본값이다. 목적이 속도이므로 Step 3의 튜닝 대상이다.

### GPU / 빌드 (가장 큰 리스크 — 이미 해소됨)

- `~/sdl_ws/src/sdl_project/TAMP/cuTAMP/curobo`에 cuRobo **v0.7.8**(SHA `d64c4b00…`)이 벤더링돼 있고,
  `cuobjdump -lelf`로 확인한 결과 `curobolib/*.so`가 **`sm_120.cubin`으로 컴파일**되어 있다.
  conda `sdl`(py3.10.19, torch 2.7.0+cu128)에서 import되고 RTX 5070을 잡는다.
- 그쪽 `tamp_server.py`는 **rclpy + curobo + torch를 한 프로세스**에서 쓴다
  (`ros2 run`으로 실행, ROS는 시스템 Humble). → "Humble 노드 안에서 cuRobo 돌리기"는 이 PC에서 검증 완료.
- `~/INTACT/PLAN.md`에도 같은 기록이 있다: Blackwell sm_120에서 curobo 커널이 안 돌아
  **torch 2.7.1+cu128로 재빌드해 해결**.
- numpy 2.x + Humble rclpy 조합도 그 환경(numpy 2.2.6)에서 이미 돌아간다.

### 라이선스 (판단 필요, 진행 전제)

- **cuRobo v0.7.x = NVIDIA License, §3.3에 non-commercial 사용 제한.**
  Apache-2.0인 것은 v0.8.0(cuRoboV2)과 `isaac_ros_cumotion*` 래퍼뿐이다.
- Humble 경로는 필연적으로 v1 코어 위에 서므로 이 제한을 상속한다.
  **연구용 전제로 진행한다.** 상용화·기술이전 계획이 생기면 그 시점에 재판단(§5).

---

## 1. 설계 결정

### D1. cuMotion은 **task(포즈) 목표 전용**. joint 목표는 OMPL 유지

cuMotion의 가장 큰 의미론 함정은 joint goal을 포즈로 바꿔 푸는 것이다(§0). 그런데 이 프로젝트는
이미 목표를 두 액션으로 갈라 놨으므로, **그 경계를 그대로 쓰면 함정을 만나지 않는다.**

| Cho 액션 | 목표 성격 | 파이프라인 |
|---|---|---|
| `moveit_task` (TaskSpace) | 포즈 | **cuMotion** — cuRobo 네이티브, 변환 손실 없음 |
| `moveit_joint` (JointSpace, home 프리셋) | 정확한 관절각 | **OMPL** — 관절각이 그대로 지켜져야 함 |

검증 가드를 덧붙여 막는 대신 애초에 마주치지 않게 한다.

**2026-09-08 측정으로 확인됨.** 설계 당시에는 소스를 읽은 추론이었는데, 좁은 씬(얇은 판)에서
방향이 정확히 갈렸다:

| 목표 종류 | cuMotion | OMPL |
|---|---|---|
| task(pose) | **27/27** | 23/27 |
| joint | 0/9 | **3/9** |

joint 목표가 지는 이유도 예상대로다 — FK 로 포즈로 변환되면서 장애물을 피할 자유도가 사라진다.
pose 목표가 이기는 이유는 예상 밖이었다: cuRobo 가 **충돌을 인식하는 IK** 를 병렬로 풀어
장애물을 피하는 해 분기를 찾는 반면, MoveIt 의 기본 IK 는 충돌을 모른 채 한 해를 내놓고
거부당한다(무작위 재시작이라 같은 목표를 시행마다 놓쳐 2/3 가 된다).

### D2. 로봇별 opt-in, 기본값은 OMPL

`cho_robot_config/config/<robot>.yaml`의 `moveit:` 블록에 파이프라인을 선언한다. 이 저장소가 이미
컨트롤러·액션을 로봇별 yaml로 등록하는 방식 그대로라 새 개념이 생기지 않는다.
선언이 없으면 기존과 100% 동일하게 동작해야 한다.

### D3. 파이썬 환경은 **venv** (`~/ros2_ws/.venv-curobo`, **python3.10**)

rclpy는 venv 안이 아니라 `/opt/ros/humble`의 PYTHONPATH에서 온다 — sdl_project가 conda로 증명한 구조다.
시스템 python을 더럽히지 않고, 디렉터리를 지우면 원복된다.
대안이었던 시스템 `pip install --user`는 런치가 단순해지는 대신 시스템 파이썬에 torch가 얹힌다.

**3.10은 선택이 아니라 강제다.** Humble rclpy는 `_rclpy_pybind11.cpython-310-*.so`이므로
3.11/3.12 인터프리터는 이 확장을 읽지 못한다. 시스템 python이 3.10.17이라 그대로 쓰면 된다.
버전 요구는 어느 쪽에서도 충돌하지 않는다: cuRobo v1(v0.7.8)은 Python 3.8+,
cuRoboV2 문서도 **Python >= 3.10**(3.13 초과 미검증)이며, PyTorch cu128 휠은 cp39~cp313을 모두 제공한다.

**실측 검증 (2026-09-08).** `python3.10 -m venv`로 만든 맨 venv에서 ROS를 source한 뒤:

| 항목 | 결과 |
|---|---|
| `import rclpy` + 노드 생성/종료 | OK |
| `moveit_msgs.action.MoveGroup` | OK |
| `sensor_msgs` / `trajectory_msgs` | **numpy 없으면 실패** → venv에 numpy 필요 |
| numpy 2.2.6 설치 후 전체 | OK |

즉 venv에 **numpy를 반드시 넣어야 한다**(cuRobo가 요구하므로 자연히 들어온다).
numpy 2.x와 Humble rclpy 조합은 여기서도, `~/sdl_ws`의 conda 환경(numpy 2.2.6)에서도 정상이다.

### D4. `extern/` 벤더링 정책을 따른다

- `extern/curobo` — 상류 v0.7.8, 태그 + SHA(`d64c4b005459db10c5dd867d8b30a87d5bda9bdb`) 고정
- `extern/isaac_ros_cumotion` — `release-3.2`, **5개 패키지 allowlist**:
  `isaac_ros_cumotion`, `_interfaces`, `_python_utils`, `_robot_description`, `_moveit`
  (benchmark / examples / segmenter / object_attachment 제외)
- 각각 `VENDORED.md`에 provenance와 **라이선스 제한**을 명시한다.

### D5. 범위 밖 (처음부터 못 박는다)

- **OpenArm 양팔(14축)** — `plan_single`은 EE 하나만 받는다. 팔별로 쪼개면 협조 계획이라는 목적이 사라진다.
- **파지 상태 계획** — attached object가 반영되지 않는다. 물체를 든 채 하는 계획은 OMPL로 남긴다.
- **실기** — 시뮬에서 판정이 끝나기 전에는 붙이지 않는다.

### D6. 상류 무패치 원칙

`~/sdl_ws` 벤더본의 `motion_gen.py` 패치(`update_pose_cost_metric()`이 hold_partial_pose 검증을
통과시키도록 `True` 반환)는 **가져오지 않는다.** 그쪽 `VENDORED.md`도 "그 브랜치에 도달하는 모든
caller에 적용되며 transfer 태스크에 한정되지 않는다"고 스스로 경고한다. 그쪽엔 fail-closed 가드가
따로 있지만 우리에겐 없다.

---

## 2. 단계

### Step 0 — OMPL 기준선 측정 **(설치 없음. 여기서 결론이 날 수도 있다)**

목적이 속도이므로 **비교 대상 없이 시작하지 않는다.** 지금 상태에서 먼저 잰다.

- 대상: FR5(Gazebo 또는 MuJoCo). goal 세트는 `cho_robot_config/config/fr5.yaml`의 `motions.reach` 프리셋 +
  워크스페이스 내 랜덤 포즈 N개(고정 시드).
- 씬 3종: (a) 빈 씬 (b) 바닥만 (현행 static scene) (c) 바닥 + 장애물 1개
- 측정: `MoveGroup.Result.planning_time` 중앙값·p95, 성공률, 실패 사유 분포
- 산출물: `todo/curobo_bench/ompl_baseline.csv` + 요약

**판정**: 계획 시간이 이미 태스크 사이클에서 무시할 수준(예: 중앙값 < 50 ms이고 전체 사이클의 5% 미만)이면
**여기서 중단한다.** 속도가 목적인데 벌 게 없다는 뜻이다.

#### 실측 결과 (2026-09-08) — **중단 조건 충족**

FR5 / Gazebo / `todo/curobo_bench/bench_planning.py` / 190회 계획. 상세는
`todo/curobo_bench/README.md`, 원자료는 `ompl_baseline.csv`.

| 씬 | 목표 | 성공 | 계획 중앙값 | p95 | 최대 | 궤적 | 계획 비중 |
|---|---|---|---|---|---|---|---|
| floor | joint | 15/15 | 5.6 ms | 14.4 ms | 15.1 ms | 5.21 s | **0.11%** |
| floor | pose | 55/55 | 15.2 ms | 32.6 ms | 228.1 ms | 6.57 s | **0.23%** |
| floor_obstacle | pose | 52/55 | 25.0 ms | 46.4 ms | 226.7 ms | 12.46 s | **0.20%** |

장애물 배치를 세 번 바꿔 재도 **중앙값이 15~25 ms에서 움직이지 않는다.**
장애물을 조이면 성공률만 떨어진다 — 목표가 실현 불가능해질 뿐 계획이 어려워지지 않는다.
6축 팔에 열린 작업공간은 RRTConnect에게 쉬운 문제다.

기준은 "중앙값 < 50 ms & 사이클의 5% 미만"이었는데 실제는 **15~25 ms / 0.2%**로 25배 여유가 있다.
가장 느린 단일 계획(228 ms)조차 6.6 s 궤적의 3.5%다. **cuMotion이 계획 시간을 0으로 만들어도
사이클은 0.2% 짧아진다.** 따라서 "플래닝 속도"를 목적으로 하는 한 A안은 진행하지 않는다.

다만 데이터가 짚은 두 지점은 남는다:

1. **실패 지연 5.02 s** — 도달 불가 목표에서 `allowed_planning_time`을 전부 소진한다. 이 데이터셋에서
   가장 큰 숫자다. cuRobo는 IK를 먼저 풀어 훨씬 빨리 포기할 가능성이 높지만, 그보다 먼저
   `allowed_planning_time`을 줄이는 쪽이 싸다.
2. **사이클을 지배하는 것은 궤적 길이(6.6~12.5 s)이고 이는 플래너가 아니라
   `max_velocity_scaling_factor: 0.25`가 정한다.** 사이클 단축이 목적이라면 지렛대는 여기다.
   cuMotion도 스케일링을 `time_dilation_factor`로 받으므로 0.25면 똑같이 4배 늘어진다.
3. **장애물 씬에서 궤적이 2배로 길어진다**(6.6 → 12.5 s). 계획 시간은 그대로인데 우회가 길어진 것이다.
   cuRobo의 궤적 최적화가 값을 낼 자리가 있다면 여기다.

**2026-09-08 후속 — 위 판정은 쉬운 씬만 본 것이었다. Step 1~3 은 실제로 완료됐고,
결론은 "목표 종류와 씬 난이도에 따라 승자가 갈린다"로 바뀌었다.**

좁은 통로 씬(얇은 판)에서 task(pose) 목표는 **cuMotion 27/27 대 OMPL 23/27**로 cuMotion 이
이긴다. 기전은 속도가 아니라 **충돌을 인식하는 IK** 다 — OMPL 은 목표 자세를 만들지 못해
0.02 s 에 즉시 거부하고, 무작위 재시작 탓에 같은 목표를 시행마다 놓친다(2/3).
joint 목표는 반대로 OMPL 이 이긴다(3/9 대 0/9): 플래너 노드가 FK 로 포즈로 변환하면서
판을 피할 자유도가 사라진다. **이것이 정확히 아래 D1 의 배선이며, 이제 측정이 뒷받침한다.**

또한 "계획 시간 244 ms" 를 "cuMotion 이 16~43배 느리다"로 적었던 것은 **틀렸다.**
cuRobo 자체는 54 ms 이고 나머지 190 ms 는 플러그인이 planning scene 전체를 매 요청마다
직렬화하고 cuRobo 월드를 재구축하는 비용이다.

그리고 **home1 로는 어려운 씬을 만들 수 없다** — 팔이 작업공간 중앙에 접혀 있어(수평 반경
0.225 m) 근처의 모든 장애물이 시작 자세와 충돌한다. 어려운 씬 측정은 `upright` 자세에서만
가능했다. 상세와 원자료는 `todo/curobo_bench/README.md`.

### Step 1 — venv + cuRobo 빌드 (30분 + 컴파일 20~40분)

- `python3.10 -m venv ~/ros2_ws/.venv-curobo`
- `pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128`
- `extern/curobo` (v0.7.8, SHA 검증) → `pip install -e . --no-build-isolation`
  - 아키텍처는 torch에서 상속되므로 sm_120으로 자동 컴파일된다.
- `VENDORED.md` 작성 (provenance + NVIDIA License non-commercial 명시)

**검증**: cuRobo 자체 예제로 plan 1회 성공 + `python -c "from curobo.wrap.reacher.motion_gen import MotionGen"`
**실패 시**: A안 전체 중단. (가능성 낮음 — §0에서 이미 검증됨)

### Step 2 — cumotion 소스 빌드 (1~2시간)

- `ros-humble-isaac-ros-common`, `ros-humble-nvblox-msgs`를 apt로 (Isaac ROS 저장소 등록 필요).
  `nvblox_msgs`는 `cumotion_planner.py`가 무조건 import하므로 필수다.
- `extern/isaac_ros_cumotion` 5개 패키지 allowlist 빌드. OpenArm 벤더 빌드 스크립트와 같은 방식으로
  검증 스크립트를 붙인다.

**검증**: `move_group`이 `isaac_ros_cumotion_moveit/CumotionPlanner`를 pluginlib으로 로드
(액션 서버가 없어 타임아웃 나는 것까지는 정상).

### Step 3 — FR5 최소 통합 + **벤치마크 판정** (반나절)

첫 타깃이 FR5인 이유 (2026-09-08 변경, 이전 초안은 UR5e였다):

- **XRDF가 이미 있고 이름이 정확히 일치한다** (§0). XRDF 작성 하루가 통째로 사라진다
- 지금 실제로 통합 중인 로봇이다 (`todo/FR5_TODO.md`), MoveIt 백엔드가 gz/mujoco/isaac/real 4종 다 있다
- `world → base_link`가 identity라 계획 프레임 불일치가 없다

손댈 파일:

| 파일 | 변경 |
|---|---|
| `cho_moveit/cho_moveit_fr5/config/fr5.xrdf` | 신규 — dcp `fr5.xrdf` 기반, `add_frame: attached_object` 추가 |
| `cho_moveit/cho_moveit_fr5/config/isaac_ros_cumotion_planning.yaml` | 신규 (상류 config 복사) |
| `cho_moveit/cho_moveit_fr5/launch/move_group.launch.py` | `pipelines=['ompl', 'isaac_ros_cumotion']` |
| `cho_moveit/cho_moveit_fr5/launch/moveit_rviz.launch.py` | 동일 |
| (launch) `cumotion_planner` 노드 추가 | `urdf_path`(xacro 확장본), `robot`(XRDF 절대경로), `tool_frame:=wrist3_link`. venv 파이썬으로 띄우는 래퍼 필요 (`Node`로는 인터프리터 지정 불가) |

첫 스모크는 dcp의 평문 `fr5.urdf`를 그대로 `urdf_path`에 넣어 배선만 확인하고,
그 다음 우리 xacro 확장본으로 바꿔 마운트·그리퍼 옵션까지 정합시킨다.

속도 튜닝: XRDF의 `acceleration_limits`/`jerk_limits`가 균일 기본값이므로,
FR5 실제 한계(`todo/FR5_TODO.md` §1의 관절·토크 한계)에 맞춰 조정한 전후를 벤치마크에 함께 남긴다.

**검증**: RViz MotionPlanning에서 파이프라인만 바꿔 플랜 성공 → Step 0과 **동일한 goal 세트·씬**으로 재측정.

**판정 기준** (여기서 채택/폐기가 갈린다):

| | 채택 | 폐기 |
|---|---|---|
| 성공률 | OMPL 이상 | OMPL 미만 |
| 계획 시간 중앙값 | **2배 이상 개선** | 2배 미만 |
| 실행 궤적 | 컨트롤러가 문제없이 추종 | 저크·불연속 발생 |

폐기 시 파이프라인 등록만 제거하면 원복된다. 벤치마크 결과는 `todo/curobo_bench/`에 남긴다.

### Step 4 — 두 번째 로봇 XRDF (UR5e는 반나절, FR3는 하루)

- **UR5e**: 상류 `ur5e.xrdf`가 우리 조인트명·tip link(`tool0`)와 일치하고
  `cho_description_ur/urdf/ur5e.urdf` 평문 URDF도 있다. `attached_object` 프레임만 추가하면 된다.
- **FR3**: 상류 `franka.xrdf`가 `panda_*` 이름이라 재사용 불가 → 새로 작성.
  스피어는 Isaac Sim 6.0의 `isaacsim.robot_setup.xrdf_editor`로
  (Isaac Sim python은 여기서만 쓴다 — 산출물이 YAML 텍스트라 ABI 무관).
- 로봇마다 필요한 것: cspace(가속·저크 한계), collision spheres, self_collision ignore,
  **`attached_object` 프레임(`add_frame` modifier)**. `tool_frames`는 노드 파라미터로 대체 가능.
- **계획 프레임 확인 필수**: `world`와 XRDF base가 다르면 포즈가 조용히 어긋난다.
  다르면 `set_base_frame` modifier로 맞춘다.

### Step 5 — 브리지 배선 + 테스트

- `cho_moveit/cho_moveit_common/scripts/moveit_action_bridge.py:374`의
  `goal.request.pipeline_id = 'ompl'` 하드코딩 → 파라미터화, **task 서버에만** cuMotion 적용 (D1)
- `cho_robot_config` 레지스트리에 파이프라인 선언 + 스키마/테스트 추가
- 회귀 테스트: 선언이 없는 로봇은 기존과 완전히 동일하게 동작할 것

---

## 3. 리스크와 함정

| 항목 | 내용 | 대응 |
|---|---|---|
| **콜리전 스피어 튜닝** | 모든 출처가 여기서 성패가 갈린다고 말한다 | FR5는 기존 XRDF 재사용(§0)으로 회피. 나머지 로봇은 Step 4에서 로봇당 반나절~하루 |
| warmup | 첫 플랜에 수십 초 | 벤치마크에서 제외, 런치 시 워밍업 |
| GPU 메모리 | 장애물 복잡도 제한 | 씬 (c)에서 확인 |
| 프레임 불일치 | 노드가 변환을 안 한다 | 로봇마다 base frame 확인 (Step 4) |
| `move_group` 동시 실행 | CUDA 에러 | 현재 구조상 로봇당 하나 — 유지 |
| libstdc++ | conda/venv가 시스템보다 구버전이면 GLIBCXX 에러 | 필요 시 시스템 libstdc++ `LD_PRELOAD` |
| 동결 가지 | release-3.2 이후 Humble 갱신 없음 | §5 |

---

## 4. 하지 않기로 한 것과 그 이유

- **Isaac Sim 번들 python 사용** — Isaac Sim 6.0.1은 py3.12, Humble rclpy는 cp310 전용이라 import 불가.
  Isaac Sim은 `curobo`(v1)를 아예 안 들고 있고 `cumotion` 1.1.0a2(cuRoboV2 세대, cp312 .so)만 번들하며
  torch도 없다. 우회하려면 `~/sdl_ws/src/sdl_project/ros2_isaacsim_ws/build_interfaces.sh`처럼
  rosidl 툴체인을 cp312 ABI로 강제해야 하는데, 그건 **커스텀 인터페이스 2개** 빌드용 장치였다.
  게다가 `move_group`이 apt cp310 세계의 C++ 노드라 A안의 전제 자체가 깨진다.
- **B안(cuRobo 전용 ROS 노드)** — `moveit_action_bridge`가 플래너 래퍼가 아니라
  씬 게이트(`/static_scene_ready`) + 컨트롤러 전환 트랜잭션 + `blocked_home_joint_goals` +
  취소 전파를 함께 들고 있다. B안은 이 층을 다시 짓거나 우회해야 한다. A안은 플래너만 갈아끼운다.
- **`curobo-core` deb 사용** — §0 참조.
- **cuTAMP / sdl 벤더본 코드 반입** — 라이선스 제한 + D6.

---

## 5. 나중에 (Jazzy / cuRoboV2)

Humble 경로는 동결된 가지다. 워크스페이스를 Jazzy로 옮길 때 cuRobo도 함께 재판단한다.

- cuRobo **v0.8.0(v2)은 Apache-2.0** → non-commercial 제한이 풀린다
- `isaac_ros_cumotion` main(4.6.0)이 그 세대다. `planning_plugins`(복수) + 신형 request adapter로 바뀌었다
- **Jazzy 4.6은 배관이 완전히 다르다.** `ros-jazzy-isaac-ros-cumotion` 4.6.0의 의존성은
  `libassimp / libeigen3 / libyaml-cpp / nitros / rclcpp / rclcpp-action`이고 **`curobo-core`(python)
  의존이 없다** — C++ 네이티브 노드로 다시 쓰였다. 즉 그쪽엔 torch도, venv도, 파이썬 플래너 노드도 없다.
- 따라서 이월되는 자산과 버려지는 것이 갈린다:
  - **이월**: 로봇별 XRDF(포맷 동일), 벤치마크 하니스와 기준선 데이터, 파이프라인 배선 설계(D1/D2)
  - **소멸**: Step 1의 venv + torch + cuRobo 소스빌드 배관 (필요가 없어진다 — 좋은 방향의 소멸)
- 공개 래퍼 [Lab-CORO/curobo_ros](https://github.com/Lab-CORO/curobo_ros)(Apache-2.0, v0.8, Jazzy+Docker)가
  설계 참고서가 된다 — unified_planner 하나에 planner 5종, 오브젝트 attach, RViz 6-DOF 마커, 테스트 스위트
- **다만 Jazzy 이전 자체의 비용이 cuRobo 통합보다 훨씬 크다.** Humble에 핀된 벤더 소스
  (`franka_ros2`, `mujoco_ros2_control`, `fairino_hardware_v3_8_0`, `openarm_can`/`openarm_hardware`),
  컨트롤러 패키지 전체, MoveIt 2.5 → Jazzy API 변화를 모두 끌고 가야 한다.
  cuRobo를 이유로 이전을 결정하지 않는다. 이전이 다른 이유로 결정되면 그때 cuRobo도 함께 올린다.

---

## 참고

- 상류: [isaac_ros_cumotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion) (`release-3.2`),
  [NVlabs/curobo](https://github.com/NVlabs/curobo) (`v0.7.8`)
- 실무 보고: [Black Coffee Robotics — cuRobo and ROS2](https://www.blackcoffeerobotics.com/blog/curobo-nvidia-and-ros2-for-motion-planning)
  (UR10 3개 씬에서 cuMotion v1 성공률 100%, 계획시간 0.134–0.301 s)
- 로컬 선례: `~/sdl_ws/src/sdl_project` (`scripts/run_tamp.sh`, `TAMP/cuTAMP/curobo/VENDORED.md`),
  `~/INTACT/PLAN.md` (sm_120 재빌드 기록)
