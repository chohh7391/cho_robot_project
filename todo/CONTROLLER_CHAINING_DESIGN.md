# 제어기 chaining — 기준 생성기와 제어 법칙의 분리

> 목적: goal 종류(JointSpace / TaskSpace / VLA / FJT)마다 다른 제어기로 전환하는 지금 구조를,
> **로봇·제어 모드마다 법칙 하나 + 그 앞에서 교체되는 기준 생성기**로 나누는 ros2_control chaining
> 구조로 옮길지 판단하기 위한 설계 문서다.
>
> 프로토타입: [`cho_controller_chain`](../cho_controller/cho_controller_chain/README.md) (Franka MuJoCo
> torque, 기존 제어기 옆에서 손으로 load). 전체 마이그레이션은 하지 않았다. 기존 제어기와의 비교는 §7.
> 대상 버전: ROS 2 Humble, controller_manager 2.54.3.

---

## 0. 요약

- 지금은 제어기 하나가 action server(기준 생성) + 궤적 + 제어 법칙을 다 가진다. 태스크 트리는 goal
  종류가 바뀔 때마다 제어기를 바꾸고, 바꿀 때마다 **법칙·게인·기준이 함께 바뀐다**. Franka 의 표준 미션은
  성공 경로에서 3번 전환한다(§1.2).
- 같은 법칙이 goal 종류 × 로봇마다 다시 구현돼 있다: JointSpace 를 서비스하는 제어기 10개, TaskSpace 8개,
  VLA 2개(§1.4). action·궤적·IK 는 이미 `cho_controller_base`/`cho_controller_common`/`cho_vla_core` 에
  하나로 모였고, 남은 중복은 법칙 × goal 종류의 곱과 로봇별 base class 다.
- 목표: 법칙은 chainable 제어기 하나가 관절마다 position·velocity·acceleration reference interface 를
  내보내고, 생성기는 그 interface 를 command 로 잡는다. goal 종류가 바뀌면 **생성기만** 바뀐다.
- Humble 2.54 는 법칙이 chained mode 에 **들어가거나 나올 때 법칙을 재시작**하고, **한 요청 안의 생성기
  교체는 재시작하지 않는다**(§3). 그래서 운영 중 법칙은 늘 chained, 아무 생성기도 없을 때는 hold 생성기,
  교체는 STRICT 한 요청이다.
- 마이그레이션은 6단계, 단계마다 측정 가능한 통과 기준을 둔다(§5). 1단계(이 프로토타입)의 측정은 §7.

---

## 1. 지금 구조와 비용

### 1.1 제어기 하나 = 생성기 + 법칙

arm 제어기는 로봇 base class(`FrankaBaseController`, `FR5BaseController`, `URBaseController`,
`OpenArmBaseController`: Pinocchio 모델, 실시간 상태, `~/controller_state`·`~/ee_state`)를 상속하고,
`update()` 안에서 (1) action server 의 궤적을 샘플하고 (2) 자기 법칙으로 command 를 쓴다. 예:
Franka `JointSpaceImpedanceController::update()` 는 Ruckig 샘플을 `q_arm_des` 로 받고
`tau = Kp (q_des - q) + Kd (v_des - dq_filtered) + nle` 를 쓴다(Kp 단위 N m/rad).

활성화 때 모든 제어기는 **측정값에서** 기준을 시작한다(`FrankaBaseController::on_activate()`:
`q_arm_des = q_arm`, `H_ee_init = H_ee`). goal 은 제어기의 비활성화를 넘어 살아남지 않는다
([CONTRACT.md, Outcomes](../cho_interfaces/CONTRACT.md#outcomes)).

### 1.2 제어기 전환 횟수

`cho_task_manager` 의 모든 트리를 레지스트리 설정으로 만들어 `SwitchControllerServiceBehavior` 와
motion leaf(JointSpace / TaskSpace / FJT action, VLA 완료 대기)를 센 결과(2026-10-14, HEAD 21892f7).
"미션 중 전환"은 성공 경로의 전환이고, `guarded_mission()` 의 abort 가지가 실패 때 한 번 더 전환한다
(hold 제어기로).

| 로봇 | 태스크 | 미션 중 전환 | 전환 순서 |
| --- | --- | --- | --- |
| franka | `pick_place` | 3 | joint_space_impedance → vla → joint_space_impedance |
| franka | `pick_place_position` | 3 | joint_space_position → vla → joint_space_position |
| franka | `tag_reach` | 3 | joint_space_impedance → task_space_qp → joint_space_impedance |
| franka | `peg_insert`, `gear_mesh`, `nut_thread` | 3 | joint_space_impedance → task_space_qp → vla |
| franka | `trajectory_replay` | 3 | joint_space_position → moveit_joint_trajectory → joint_space_position |
| franka | `controller_check_torque` / `_position` / `_velocity` | 7 / 3 / 4 | 제어기를 하나씩 모두 |
| fr5 | `fjt_handover`, `trajectory_replay` | 3 | joint_space_position ↔ joint_trajectory |
| ur5e | `pick_place`, `multi_move`, `trajectory_replay` | 3 | joint_space_position → task_space_ik (또는 JTC) → joint_space_position |
| openarm | `controller_check_torque` / `trajectory_replay` / `mit_task_tuning` | 1 / 3 / 0 | |

전환은 모두 exclusive 다: 레지스트리의 arm 제어기 집합(`exclusive_arm_controllers()`: Franka 12개, FR5 3개,
UR 3개, OpenArm 7개)에서 켤 것을 뺀 전부를 BEST_EFFORT 로 끈다. 트리는 goal 종류마다 어느 제어기가 그
action 을 서비스하는지 알아야 하고(레지스트리 role: `direct_joint`, `direct_task`, `vla`,
`hold_by_control_mode`, `additional_arm`), 그 지식이 트리와 레지스트리에 흩어진다.

### 1.3 전환 한 번의 비용

1. **법칙이 바뀐다.** joint impedance(Kp 400/100 N m/rad, Kd 10/2) → task-space QP → VLA 의 Cartesian
   impedance(kp_task 565/28)처럼 법칙의 모양과 게인이 같은 순간에 바뀐다. 전환 순간의 토크 불연속은
   같은 상태에서 두 법칙 출력의 차이다.
2. **기준이 측정값으로 리셋된다.** 새 제어기는 측정 자세에서 시작하므로 나가는 제어기가 쥐고 있던 추종
   오차(중력·마찰 아래의 정상상태 오차)를 버린다. 기준이 측정값으로 한 번 뛴다.
3. **전환은 goal 의 경계다.** 나가는 제어기의 goal 은 abort 되고, 트리는 goal 종류가 바뀔 때마다 switch
   leaf 와 그 실패 경로를 가진다.
4. **OpenArm MIT 에서는 전환이 프로토콜 사건이다.** arm 을 시작·정지하는 전환은 hardware 의 `SwitchGate` 가
   measured SAFE 로 넣는다([cho_openarm_mit_core/DESIGN.md, SwitchGate](../cho_hardware/cho_openarm_mit_core/DESIGN.md#switchgate)).
   goal 종류를 바꿀 때마다 SAFE 를 한 번 거친다.

전환 과도의 크기는 §7 에서 MuJoCo 로 쟀다.

### 1.4 로봇별 중복

`src/` + `include/` 줄 수(wc -l, 2026-10-14). 생성기와 법칙을 한 클래스에 묶은 제어기만 센다.

| 법칙 + goal 종류 | Franka | FR5 | UR | OpenArm |
| --- | --- | --- | --- | --- |
| 관절 position + JointSpace | `JointSpacePositionController` 182 | 144 | 125 | legacy 160 |
| DLS IK position + TaskSpace | `TaskSpaceIKController` 277 | 627 | 207 | — |
| 관절 impedance(torque) + JointSpace | `JointSpaceImpedanceController` 219 | — | — | legacy 391, MIT `JointImpedanceActionController` (`direct_controller.cpp`) |
| 관절 velocity + JointSpace | `JointSpaceVelocityController` 324 | — | — | legacy 204 |
| Cartesian impedance + TaskSpace | `TaskSpaceImpedanceController` 278 | — | — | MIT 1965 (drive-side 법칙) |
| QP / OSC + JointSpace·TaskSpace | `JointSpaceQPController` 340, `TaskSpaceQPController` 752, `OperationalSpaceController` 289 | — | — | — |
| VLA host | `VLAController` + `VLAActionServer` 1845 | — | — | MIT `VlaController` 1301 |
| 로봇 base class | 933 | 510 | 480 | 864 |

- JointSpace action 을 서비스하는 제어기 10개(Franka 4, FR5 1, UR 1, OpenArm legacy 3, MIT 1), TaskSpace 8개
  (Franka 5, FR5 1, UR 1, MIT 1), VLA 2개. 20개 클래스가 각자 생성기 하나와 법칙 하나를 묶는다.
- 이미 하나로 모인 것: action 처리(`GoalPhaseActionServer`, `JointSpaceServer`, `TaskSpaceServer`; 로봇별
  `servers/` adapter 는 35–56줄), DLS IK(`kinematics.hpp` `task_ik_step()`), Ruckig 궤적
  (`cho_controller_common`), VLA 파이프라인(`cho_vla_core::Session`).
- 남은 중복: 같은 법칙이 goal 종류마다(Franka joint impedance 는 JointSpace 에만, Cartesian impedance 는
  TaskSpace 에만 붙는다), 같은 base 기능(robot_description 가져오기, 모델, `nominal_period()`, torque clip,
  상태 토픽)이 로봇마다 4번.
- 플러그인 수: Franka 13, FR5 2, UR 2, OpenArm 15(legacy 4 + MIT 11), 그리퍼 1.

---

## 2. 목표 구조

### 2.1 법칙, 생성기, reference interface

```
 JointSpace action ──► joint_space_generator ──┐
 TaskSpace action  ──► task_space_generator  ──┤ (한 번에 하나)     <law>/<joint>/position
 ActionChunk       ──► vla_generator         ──┼──────────────────► <law>/<joint>/velocity
 FollowJointTraj.  ──► fjt_generator         ──┤                    <law>/<joint>/acceleration
 (goal 없음)       ──► hold_generator        ──┘                              │
                                                                             ▼
                              <joint>/position, velocity (state) ──► law ──► <joint>/effort (또는 position, velocity)
```

- **법칙**: 로봇·제어 모드마다 chainable 제어기 하나. arm 의 command interface 를 잡고, 관절마다
  `<law>/<joint>/position`, `/velocity`, `/acceleration` reference interface 를 내보낸다. 게인, 토크 한계,
  rotor inertia, 중력 보상은 법칙의 파라미터다. torque 법칙은 프로토타입의
  `tau = M(q) (a_ref + Kp (q_ref - q) + Kd (v_ref - dq)) + h(q, dq)`. position 법칙은 `q_ref` 를 한계·slew
  guard 를 거쳐 그대로, velocity 법칙은 `v_ref + Kp (q_ref - q)`.
- **생성기**: 법칙의 reference interface 를 command 로 잡는 보통 제어기. action 하나를 서비스한다. 로봇은
  `robot_description` 과 관절 이름으로만 들어온다(Pinocchio reduced model). goal 이 없으면 마지막 기준을
  hold 한다.
- **관절 수준 기준인 이유**: 모든 생성기는 결국 관절 기준을 만든다(TaskSpace 는 DLS 한 걸음, VLA 의 task
  chunk 도 IK, FJT 는 원래 관절). 그러면 법칙 하나가 모든 goal 종류를 받는다. 대가는 Cartesian 컴플라이언스다:
  지금 Cartesian impedance·OSC·TSQP 가 주는 작업공간 강성은 관절 impedance 로는 관절공간 강성이 된다(§6).
  task 수준 법칙(pose 7 + twist 6 + acceleration 6 reference)은 접촉 태스크가 필요로 할 때 둘째 법칙으로 둔다.

### 2.2 프로토타입이 정한 것

1. **생성기는 활성화 뒤 첫 update 에서 hold 를 잡는다**(on_activate 가 아니라). 그때는 같은 switch 의 모든
   on_activate(진입으로 재시작되는 법칙의 것 포함)가 끝났고, 그 cycle 에서 생성기가 법칙보다 먼저 update
   하므로(§3.1 5번) 읽는 순서에 기대지 않는다. hold = 법칙의 reference 의 position 이 모두 유한하면 그 값,
   아니면 측정 q; v = a = 0. 그래서 생성기 교체가 기준 위치를 뛰게 하지 않는다.
2. **법칙은 configure 에서 reference 를 NaN 으로 채우고 activate 에서 hold 를 써 넣는다.** 0.0 이 자세로
   읽히는 일이 없다. 유한하지 않은 reference 묶음은 마지막 유한한 묶음으로 바뀐다.
3. **TaskSpace 생성기는 acceleration 0 을 쓴다.** 관절 기준은 cycle 마다 `max_joint_velocity` 로 자른 DLS
   한 걸음이라, 그 차분을 acceleration 으로 쓰면 M(q) 를 거쳐 토크 스파이크가 된다. JointSpace 생성기는
   Ruckig 샘플의 acceleration(jerk 로 묶여 매끄럽다)을 쓴다.
4. **게인 단위는 1/s², 1/s 이고 M(q) 가 곱해진다.** 같은 kp 가 모든 관절에서 같은 대역폭을 준다
   (kp 900, kd 60: 30 rad/s 임계감쇠). 그래서 M(q) 에 rotor inertia 가 있어야 한다. URDF 에는 없고 MuJoCo 는
   armature 0.195 / 0.074 kg m² 를 둔다. FR3 ready 자세에서 M 의 대각:

   | 관절 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
   | --- | --- | --- | --- | --- | --- | --- | --- |
   | M_ii, 링크만 [kg m²] | 0.568 | 1.631 | 1.029 | 0.995 | 0.042 | 0.049 | 0.002 |
   | M_ii, armature 포함 | 0.763 | 1.826 | 1.224 | 1.190 | 0.116 | 0.123 | 0.076 |
   | 등가 강성 kp·M_ii [N m/rad] | 686 | 1644 | 1101 | 1071 | 105 | 111 | 68 |
   | 등가 감쇠 kd·M_ii [N m s/rad] | 46 | 110 | 73 | 71 | 7.0 | 7.4 | 4.6 |

   armature 가 없으면 7번 관절의 등가 강성은 1.8 N m/rad 로 무너진다. 비교 대상
   `joint_space_impedance_controller` 는 Kp 400/100 N m/rad, Kd 10/2 N m s/rad 다.
5. **TaskSpace 생성기는 goal 첫 cycle 에 SE3 궤적을 FK(q_ref) 에서 다시 시작한다**(`setInitSample`,
   `cho_controller_fr5/src/task_space_ik_controller.cpp` 와 같은 방식). `TaskSpaceServer::compute()` 는 측정
   pose 에서 시작하므로 그대로면 기준이 뛴다. 상대 goal 은 측정 pose 기준 그대로다(CONTRACT 가 허용).
   성공·timeout 은 측정 pose 로 판정한다.
6. `ik_gain * dt` 는 0.5 이하로 자른다(`task_ik_step()` 은 gain·dt < 1 을 전제하지만 검사하지 않는다).
7. 법칙의 update 는 할당·lock 이 없다(RealtimePublisher 의 trylock 하나만). 생성기의 dt 는
   `ReferenceGenerator::nominal_period()` 다(CLAUDE.md, Control period).

---

## 3. Humble 2.54 의 chained mode 규칙

### 3.1 소스에서 확인한 규칙

`controller_manager.cpp` @2.54.3 과 `chainable_controller_interface.hpp` 에서:

1. chainable 제어기는 `on_export_reference_interfaces()`, `on_set_chained_mode(bool)`,
   `update_reference_from_subscribers()`, `update_and_write_commands()` 를 구현한다. **Humble 은 reference
   interface 만** 내보낸다. 법칙의 state(예: 지금의 hold)를 생성기가 interface 로 읽을 수 없다(Jazzy 부터).
2. 앞 제어기(생성기)의 활성화는 뒤 제어기(법칙)가 이미 활성이거나 같은 activate 요청에 있어야 한다
   (:2420-2428).
3. `switch_controller()` 는 먼저 `propagate_deactivation_of_chained_mode()`(:1065)로, 꺼지는 앞 제어기의
   뒤 제어기를 from_chained 목록에 넣는다. 그다음 활성화마다 `check_following_controllers_for_activate()`
   (:1087): 뒤 제어기가 이미 chained 면 from_chained 에서 **뺀다**("should stay in chained mode",
   :2463-2475), chained 가 아니면 to_chained 에 넣는다.
4. to_chained / from_chained 에 있는 활성 제어기는 **재시작**된다: 같은 switch 안에서 deactivate 후 activate
   (:1182-1188). chained 플래그는 비활성일 때만 바뀐다.
5. 갱신 순서: `controller_sorting` 으로 stable_sort(:906-909, :2569)해 앞 제어기가 뒤 제어기보다 먼저
   update 한다. 생성기가 쓴 기준을 법칙이 같은 cycle 에 읽는다(한 cycle 지연 없음).

### 3.2 요청별 결과

| 요청 | 법칙 | 결과 |
| --- | --- | --- |
| 법칙만 활성 | chained 아님 | 활성화 때 측정 자세를 hold (v = a = 0) |
| 법칙 활성 상태에서 생성기 A 활성화 | chained 진입 → **재시작** | 법칙 hold = 그때의 측정 q; A 는 첫 update 에서 그 hold 를 잡는다 |
| 법칙 + A 를 한 요청으로 활성화 | chained 로 활성화 | A 의 hold = 법칙의 hold |
| 한 요청으로 A 끄고 B 켬 | chained 유지, **재시작 없음** | B 는 A 의 마지막 기준 위치에서 시작 |
| 마지막 생성기만 끔 | chained 이탈 → **재시작** | 법칙 hold = 그때의 측정 q |
| A 끄기, B 켜기를 두 요청으로 | 이탈·진입 → 재시작 2번 | |
| BEST_EFFORT 교체에서 B 활성화 실패 | A 만 꺼짐 → 이탈 → 재시작 | |

재시작은 switch 가 도는 cycle 안에서 끝나므로 command 가 빠지는 cycle 은 없다. 대신 기준이 측정 q 로,
v_ref·a_ref 가 0 으로 바뀌어 토크가 그 순간 `M(q) (a_ref + Kp (q_ref - q) + Kd v_ref)` 만큼 뛴다. 정지 중이면
작고, 움직이는 중이면 감쇠 브레이크다.

생성기 교체에서도 B 는 v = a = 0 에서 시작한다. goal 사이(A 의 v_ref = a_ref = 0)의 교체는 연속이고,
움직이는 중의 교체는 토크가 `M(q) (a_ref + Kd v_ref)`(A 의 마지막 값)만큼 뛴다.

`cho_controller_chain/test/test_chain_switch.cpp`(controller_manager harness, mock hardware)가 위 표의
1–5행을 확인한다: 진입·이탈 때 법칙의 activation 수 +1, 교체 때 불변, 교체 뒤 법칙의 기준이 A 의 마지막
기준과 같고 effort 가 이어진다.

### 3.3 설계에 주는 결론

- **법칙은 운영 중 늘 chained 다.** bringup 이 법칙과 hold 생성기를 한 요청으로 함께 활성화하고
  (`spawner --activate-as-group`), 그 뒤로는 생성기만 바뀐다.
- **hold 생성기**: action 없이 법칙의 지금 기준을 잡고 있는 생성기. 아무 goal 도 없을 때, 그리고 abort 가
  넘어가는 곳. 프로토타입에서는 goal 없는 JointSpace/TaskSpace 생성기가 그 역할을 한다.
- **교체는 STRICT 한 요청**: 지금 활성인 생성기를 끄고 새 생성기를 켠다. BEST_EFFORT 는 B 를 켤 수 없을 때
  A 만 꺼서 법칙이 이탈·재시작하고, STRICT 는 요청 때의 검사(상태, interface)에 걸리면 요청 전체를 거절해
  ("Aborting, no controller is switched! ('STRICT' switch)") A 가 계속 돈다. B 의 `on_activate()` 자체가
  실패할 때의 동작은 확인하지 않았다. 생성기의 `on_activate()` 는 실패할 일을 configure 로 미룬다.
- 지금 태스크 매니저의 exclusive BEST_EFFORT 전환(레지스트리 집합 전체를 끄는 것)은 맞지 않는다.
  `list_controllers` 로 활성 생성기를 읽고 그 하나를 끄는 STRICT 교체로 바뀌어야 한다(§5 의 3단계).

---

## 4. 다른 부분과의 관계

### 4.1 VLA 생성기

`cho_vla_core::Session` 은 이미 ROS 없이 ingest / step / telemetry 를 나누고, host 는 ROS adapter 와 법칙만
가진다([cho_vla_core/DESIGN.md, Session](../cho_controller/utils/cho_vla_core/DESIGN.md#session)). VLA
생성기는 그 host 에서 법칙을 뺀 것이다.

- `Session::step()` 이 cycle 마다 관절 기준(joint chunk) 또는 task pose(task chunk)를 준다. task pose 는
  TaskSpace 생성기와 같은 DLS 한 걸음으로 관절 기준이 된다. action server, chunk 구독, telemetry,
  `take_gripper()` 는 생성기에 남는다.
- seed: `HostCommand` 의 `kRest` 는 host 가 지금 명령하는 자세를 받는다. 생성기는 법칙의 현재 기준을 준다
  (JointSpace / TaskSpace 생성기와 같은 규칙). 그래서 JointSpace → VLA 교체도 기준을 뛰게 하지 않는다.
- `kStopping`: Franka 는 `come_to_rest()` 의 정지 기준을 명령하고 OpenArm 은 측정 pose 로 놓는다. chain 에서는
  생성기가 정지 기준을 쓰고 법칙이 잡는다(놓지 않는다).
- Franka `vla_controller` 의 position/velocity 모드는 이미 개루프 `q_ref` 를 적분하는 생성기다. effort 모드의
  task chunk 는 지금 Cartesian impedance 로 추종하는데, chain 에서는 IK 기준을 관절 impedance 가 추종한다.
  컴플라이언스가 달라지는 것이 2단계의 측정 항목이다.

### 4.2 그리퍼

그리퍼 제어기는 손가락 interface 를 따로 잡고 exclusive 집합에도 없다. chain 밖에 그대로 둔다. VLA 생성기는
지금 host 처럼 그리퍼 제어기의 `~/gripper` action 으로 보낸다.

### 4.3 OpenArm MIT

MIT tuple `(q_des, dq_des, kp, kd, tau_ff)` 는 그 자체로 drive 안에서 계산되는 관절 수준 법칙이다:
`tau = kp (q_des - q) + kd (dq_des - dq) + tau_ff`. chain 에서 법칙 자리는 producer 다.

- 법칙(chainable producer)이 session / lease / SAFE 프로토콜(`DirectControllerBase` 의 몫)과 arm 의 39개
  command interface 를 갖고, reference 에서 `q_des = q_ref`, `dq_des = v_ref`, 관절별 고정 kp·kd,
  `tau_ff = M(q) a_ref + h(q, dq)`(profile 의 `tau_ff_slew_per_s` 로 slew)를 쓴다.
- 그러면 생성기 교체는 MIT command interface 를 건드리지 않아 `SwitchGate` 의 SAFE 전환이 goal 종류마다
  일어나지 않는다. 법칙 재시작(chained 진입·이탈)은 건드린다. 법칙이 늘 chained 여야 하는 이유가 하나 더 있다.
  확인할 것: 생성기 교체 요청이 hardware 의 `prepare_command_mode_switch()` 에 arm interface 를 하나도
  넘기지 않는지, `FakeMitSystem` harness 로.
- drive-side Cartesian 법칙(`TaskSpaceImpedanceController`, `q_des = q + J⁺ (x_des ⊖ x)`)은 task 수준
  법칙이다. 관절 법칙 아래에서는 TaskSpace 생성기의 DLS 걸음이 대신하고, Franka 와 같은 컴플라이언스 질문이
  남는다.
- `M(q)` 를 곱하는 게인은 Damiao rotor inertia(관절 3·4 에서 0.16 kg m², 링크의 약 15배)가 모델에 있어야
  의미가 있다. 지금 어떤 OpenArm 설정도 `rotor_inertia` 를 두지 않는다.
- paired FJT(`BimanualFollowJointTrajectoryController`, pair ownership token)는 따로 남는다. 두 팔을 한
  트랜잭션으로 쓰는 법칙은 이 설계 밖이다.

### 4.4 MoveIt / FollowJointTrajectory

FJT 생성기는 `control_msgs/FollowJointTrajectory` 를 받아 궤적 샘플(position, velocity, acceleration)을 쓴다.
그러면 `trajectory_replay` 의 position ↔ JTC 전환도 생성기 교체가 되고, torque 모드에서도 MoveIt 실행이
가능해진다(지금 `moveit_joint_trajectory_controller` 는 position 모드만).

---

## 5. 단계별 마이그레이션

각 단계는 앞 단계의 기준을 통과해야 시작한다. 기존 제어기는 사용자가 지우기로 정할 때까지 그대로 둔다.

| 단계 | 내용 | 통과 기준 (측정) |
| --- | --- | --- |
| 1 | **프로토타입** (이번): Franka MuJoCo torque, 법칙 + JointSpace / TaskSpace 생성기, 손으로 load | harness 테스트: 교체 때 재시작 없음, effort 연속, 진입·이탈 동작이 §3.2 와 같음. MuJoCo(§7): 같은 JointSpace goal 의 추종 오차 RMS·max 가 `joint_space_impedance_controller` 이하; JointSpace → TaskSpace 전환의 max \|Δτ\| 와 전환 뒤 0.5 s EE 편차가 기존 전환 이하; TaskSpace goal 완료 |
| 2 | **VLA 생성기**(Session 위), Franka MuJoCo | `vla_controller`(effort) 대비 같은 chunk 스트림의 추종 RMS·지연·진동; JointSpace → VLA → JointSpace 가 재시작 없이; 법칙 + 생성기 cycle CPU 시간이 `vla_controller` 이하 |
| 3 | **Franka torque 교체**: bringup 이 법칙 + hold 생성기를 활성, 레지스트리 role 을 생성기로, 태스크 매니저 전환을 STRICT 교체로(활성 생성기는 `list_controllers` 로) | Franka torque 트리 전부 MuJoCo 통과; abort 가 hold 생성기로; 실물 FR3 에서 추종·전환 측정과 forge 태스크의 접촉 거동이 기존(TSQP + VLA)과 같거나 낫다 |
| 4 | **position / velocity 법칙**, FR5·UR 의 `JointSpacePosition` / `TaskSpaceIK` 를 생성기로 | 로봇별 트리 통과; FR5 실물 125 Hz 에서 추종 오차가 기존 이하; FJT 생성기로 trajectory_replay |
| 5 | **OpenArm MIT chained producer** | `FakeMitSystem` 에서 생성기 교체가 SAFE 를 만들지 않음; 실물에서 `task_space_impedance_mit_controller` 대비 추종·링잉 |
| 6 | 교체된 제어기 제거 | 사용자 결정. 그 전에 3–5단계의 실물 확인 |

모든 단계에 공통: 새 제어기마다 harness 테스트, cycle 당 CPU 시간(법칙 + 활성 생성기 vs 바뀌는 제어기),
launch golden 과 registry 테스트 통과.

---

## 6. 위험과 열린 질문

- **Humble 과 Jazzy.** Humble 은 reference interface 만 있어 법칙의 상태를 생성기에 내보낼 수 없다(생성기는
  자기가 쓰는 reference 를 읽어 hold 를 잡는다). Jazzy(ros2_control 4.x)는 chainable 제어기의 state interface
  export 를 더한다. 진입·이탈 재시작과 정렬 규칙은 옮길 때 그 버전의 `switch_controller()` 를 다시 읽고
  판단한다.
- **Cartesian 컴플라이언스.** 관절 법칙은 작업공간 강성을 직접 주지 못한다. forge 태스크(TSQP + VLA)처럼
  접촉이 있는 경로는 2–3단계에서 측정하고, 모자라면 task 수준 법칙을 둘째 법칙으로 둔다(교체는 법칙 재시작이
  된다).
- **특이점·가속도 한계.** TSQP 의 관절 가속도 bound 같은 것이 TaskSpace 생성기에는 없다(`max_joint_velocity`
  만). 특이점 근처 goal 은 DLS damping 과 속도 제한이 전부다.
- **교체 규율.** STRICT 교체는 지금 활성인 생성기를 정확히 알아야 한다. 태스크 매니저가 그것을 읽지 못하면
  요청이 거절된다(안전한 쪽).
- **모델 의존 게인.** `M(q)` 를 곱하는 게인은 rotor inertia 와 payload 가 모델에 맞아야 한다. 프로토타입 법칙에는
  payload 파라미터(`FrankaBaseController` 의 `mass`, `center_of_mass`, `load_inertia`)가 아직 없다.
- **마찰.** MuJoCo FR3 의 frictionloss(관절 1–4 1.137 N m)를 법칙이 보상하지 않는다. 느린 구간의 추종 오차에
  섞인다. 기존 joint impedance 도 같다.
- **상태 토픽.** 법칙이 `~/controller_state` 를 내고 생성기는 아무것도 내지 않는다. `~/ee_state` 는 없다
  (`/ee_state/pose` 는 `ee_state_broadcaster`).

---

## 7. 측정 (AC5)

MuJoCo Franka torque(`bringup_mujoco_robot.launch.py control_mode:=torque`), 경로마다 새 launch, 같은 goal
(2026-10-14, 21892f7 + 이 프로토타입). 시작: `joint_space_impedance_controller` 로 home keyframe
Q0 = [0, −0.785, 0, −2.356, 0, 1.57, −0.785]. JointSpace goal G: Q0 → Q0 + [0.4, 0.3, −0.3, 0.4, 0.3, −0.3, 0.4],
3 s. 그 뒤 1 s 에 JointSpace → TaskSpace 전환(한 STRICT 요청), 0.5 s 뒤 TaskSpace goal T: G 뒤 EE 위치 +5 cm x,
−5 cm z, 자세 그대로, `fr3_link0` 절대, 3 s.

지표마다 두 경로에 같은 출처: 추종 오차 = `<ctrl>/controller_state` 의 reference − feedback 관절 노름(G 시작부터
3.5 s), 토크 = `/joint_states` effort, EE = `/ee_state/pose`. max |Δτ| = 전환 요청부터 응답 뒤 50 ms 까지 연속
표본 사이 관절별 effort 변화의 최댓값(괄호: 요청 전 0.5 s 의 같은 값). EE 편차 = 응답 뒤 0.5 s 동안 요청 때
위치에서 벗어난 최대 거리.

| 경로 | JointSpace goal 추종 오차 RMS / max [rad] | JointSpace→TaskSpace 전환 max \|Δτ\| [N m] | 전환 뒤 0.5 s EE 편차 [mm] | TaskSpace goal (끝 오차) |
| --- | --- | --- | --- | --- |
| 기존: `joint_space_impedance_controller` → `task_space_impedance_controller` | 0.0101 / 0.0116 | 2.15 (0.003) | 1.12 | 완료 (1.55 mm) |
| chain: `chain_impedance_law` kp 900 / kd 60 + `chain_joint_space_generator` → `chain_task_space_generator` | 0.0108 / 0.0119 | 0.0009 (0.002) | 0.24 | 완료 (0.06 mm) |
| chain, 법칙 kp 1600 / kd 80 | 0.0061 / 0.0070 | 0.0010 (0.002) | 0.09 | 완료 (0.00 mm) |

- 전환: chain 은 전환 순간 토크 변화가 평소 cycle 변화보다 작다(재시작 없음, B 가 A 의 기준에서 시작).
  기존 경로는 법칙이 바뀌며 2.15 N m 가 한 번에 뛴다.
- 추종: 두 경로 모두 오차의 대부분이 끝까지 남는 정상상태 오차다(끝 오차 ≈ RMS; MuJoCo frictionloss 를 둘 다
  보상하지 않는다). 관절별 RMS [rad]: 기존 [0.0025, 0.0026, 0.0026, 0.0026, 0.0072, 0.0042, 0.0024],
  chain kp 900 [0.0034, 0.0017, 0.0031, 0.0029, 0.0059, 0.0058, 0.0040]. kp 900 에서 7번 관절의 등가 강성
  (68 N m/rad, §2.2)이 기존 100 보다 낮다. kp 1600 이면 모든 관절에서 기존보다 강하고 RMS 가 40 % 작다.
  즉 차이는 게인이고, 1단계 통과 기준의 "추종 오차 이하"는 kp 900 에서는 못 미치고 kp 1600 에서 맞는다.
  설정 기본값은 kp 900 / kd 60 이다.
- 기존 제어기(`joint_space_impedance_controller`)에서 법칙 + 생성기로 넘어가는 첫 요청(법칙이 활성화되며 측정
  자세를 hold)의 max |Δτ| 는 0.84–0.89 N m, EE 편차 0.002 mm 다.
- 두 경로는 법칙과 게인이 다르다. 기존 경로의 TaskSpace goal 은 Cartesian impedance 가, chain 의 것은 IK 기준 +
  관절 impedance 가 추종한다. 이 표는 설정된 그대로의 두 경로를 비교하고, 구조만의 차이를 따로 떼지 않는다.
