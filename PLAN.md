# PLAN.md — 제어기 추적 성능 개선

> 목적: 모든 제어기가 정해진 waypoint 구간에서 current가 desired를 잘 따라가고(추적),
> 마지막에 안정적으로 수렴하도록 개선한다. 연구실 공식 코드 수준으로 다듬는다.

## 결정사항 (사용자 확정)
- **Waypoint 방식**: 구간별 point-to-point 추적 개선. (waypoint에서 멈췄다 출발하는 현 구조 유지,
  각 구간 내 추적만 개선. blend 통과는 하지 않음.)
- **적용 순서**: **Franka 먼저** → 검증 후 OpenArm / UR 확장.
- **검증 환경**: 모든 시뮬레이터에서 동작. **MuJoCo → Gazebo → Isaac Sim** 순서로 해결.
  Real robot 검증은 사용자가 직접 담당.
- **작업 방식**: 이 PLAN.md를 기준으로 진행하며, 각 단계 완료 시 체크박스 갱신.

---

## 근본 진단 (Root cause)

Cubic 트래젝토리 생성기(`TrajectoryEuclidianCubic`, `TrajectorySE3Cubic`)의 `computeNext()`가
**위치(`pos`)만 채우고 속도(`vel`)·가속도(`acc`)를 항상 0으로 둔다.**

그런데 소비하는 쪽은 피드포워드 경로가 **이미 배선**되어 있다:
- `TaskJointPosture::compute`: `a_des = -Kp·e_p - Kd·(v - v_ref) + a_ref`
- `TaskSE3Equality::compute`: `a_des = Kp·e_p + Kd·v_err + a_ref`  (v_err은 v_ref 사용)
- Franka `JointSpaceImpedanceController`: `τ = kp·e_p + kd·(v_des - dq) + nle`
- OpenArm `JointSpaceImpedanceController`: `τ = M·(kp·e_p + kd·(v_des - dq) + ki·e_i) + nle`

결과적으로 impedance / joint_qp / task_qp 계열이 **v_des=0, a_des=0으로 고정된 순수 PD**로
동작 → 모션 중 desired에 지연(lag)이 생기고 kd 항이 실제 속도를 제동. 끝점은 v_des→0이 맞아
수렴 자체는 정상. 즉 "구간 추적"만 나쁘다.

velocity / IK 계열은 위치 스트림을 수치미분해 자체 피드포워드를 만들어 이 문제를 우회한다.
`task_space_impedance` / `operational_space`는 v_des를 코드에 0으로 박아 피드포워드 경로 자체가 없다.

### Cubic 피드포워드 수식 (참고)
구간 길이 T, τ = t − t_start, d = goal − init, s(τ)=3τ²/T² − 2τ³/T³ (경계 속도 0):
- pos = init + d·s,  vel = d·ṡ,  acc = d·s̈
- ṡ = 6τ/T² − 6τ²/T³,  s̈ = 6/T² − 12τ/T³
- SE3 회전: R(t)=R_init·exp(ω̂·s), ω=vee(log(R_initᵀR_goal));
  world-aligned twist = R(t)·ω·ṡ (각속도), R(t)·ω·s̈ (각가속). (TaskSE3Equality가 기대하는 프레임)

---

## 컨트롤러별 피드포워드 소비 현황 (Franka)

| 제어기 | 인터페이스 | trajectory vel/acc 소비 | Phase 1 후 개선 | 추가 조치 |
| --- | --- | --- | --- | --- |
| joint_space_impedance | effort | vel ✅ | 자동 개선 | — |
| joint_space_qp | effort | vel ✅ / acc ❌(미복사) | 자동 + acc 복사 | qp에 acc.head 복사 |
| task_space_qp | effort | SE3 vel/acc ✅(setReference) | 자동 개선 | — |
| task_space_impedance | effort | ❌ 경로 없음 | 없음 | **Phase 2**: v_des/a_des 추가 |
| operational_space | effort | ❌ v_des=0 하드코딩 | 없음 | **Phase 2**: v_des/a_des 추가 |
| joint_space_position | position | pos만 | 해당없음(내부 PD) | — |
| task_space_ik | position | pos만(수치미분) | 해당없음 | — |
| joint_space_velocity | velocity | pos만(수치미분 ff) | 해당없음 | — |
| task_space_velocity | velocity | pos만(수치미분 ff) | 해당없음 | — |
| gravity_compensation | effort | 추적 없음 | — | — |

---

## Phase 1 — 공통 Cubic 피드포워드 (핵심, 저위험, 최대 효과)
`cho_controller_common` 한 곳 수정으로 impedance/joint_qp/task_qp가 한 번에 개선.
경계 속도가 0이라 정지/수렴 안정성엔 영향 없음 (모션 중 lag만 감소).

- [x] `trajectory_euclidian.cpp` `TrajectoryEuclidianCubic::computeNext()` — 해석적 vel/acc 추가,
      active window 밖에서는 vel/acc = 0.
- [x] `trajectory_se3.cpp` `TrajectorySE3Cubic::computeNext()` — 병진 vel/acc(world) + 회전
      각속도/각가속(world-aligned) 추가. 중복 대입 정리. (subagent 구현, 빌드 통과)
- [x] `joint_space_qp_controller.cpp` — `sample_posture.acc.head(num_dof_) = trajectory_sample.acc` 복사.
- [x] 빌드: `cho_controller_common cho_controller_franka` 컴파일 통과 (29.5s, warning은 기존 Timeopt 것뿐).

### Phase 1 독립 리뷰 결과 (감독 subagent)
수학·SE3 프레임 규약·경계 연속성·qp acc 전파·회귀 전부 **CORRECT**, 차단 버그 없음. 하드닝 3건:
- [x] (Low) 두 `computeNext`에 최소 duration 가드 `T = max(m_duration, 1e-6)`. (빌드 통과)
- [x] (Low) Euclidian: `VectorXd` 임시 3개 제거 → `m_sample`에 직접 기록(RT-clean).
- [x] (Note) SE3 각속도 ff는 `ee_offset=0` 전제 → 주석 명시(현재 전 제어기 offset 0).

**→ Phase 1 완료.** (`cho_controller_common` + `cho_controller_franka` 빌드 클린)

### 별도 정리 항목 (feedforward 범위 밖, 발견됨)
- [ ] (Minor) `TrajectoryEuclidianTimeopt::computeNext` / `TrajectorySE3Timeopt::computeNext`:
      `else { assert(false); }` 경로가 non-void 함수 끝에서 값 없이 떨어짐(`-Wreturn-type`).
      release(assert off) 시 UB 가능. 기존 잠재버그 — 공식화 시 return/throw 추가 권장.
- [x] OpenArm/UR가 공통 코드 공유 → 회귀 없음 확인(소스 분석):
      OpenArm/UR의 position·velocity·IK는 전부 `.pos`만 소비(velocity는 위치 수치미분) → 무영향.
      OpenArm impedance만 `.vel` 소비 → 홈잉 램프에서 ff 채워짐(개선 방향, 경계 0이라 안전).

## Phase 2 — 피드포워드 경로 없는 Franka 태스크 제어기
- [x] `task_space_impedance_controller.cpp` — SE3 vel(world-aligned) 읽어
      `task_wrench = kp·Δpose + kd·(v_des − ee_vel)` (idle 시 v_des=0 → 기존과 동일).
- [x] `operational_space_controller.cpp` — SE3 vel을 `R_eeᵀ`로 local 회전해
      `error_dot = v_des − v_curr` (idle 시 v_des=0 → 기존과 동일).
- [x] 빌드 확인 (cho_controller_franka 클린).
- [x] 독립 리뷰(감독): Q1–Q4 전부 CORRECT/CLEAN, 차단 이슈 없음.
      (부수: task_space_impedance 미사용 `R_spatial` 죽은 코드 — 기존, 정리 대상.)

**→ Phase 2 완료.**

## Phase 3 — 검증 (MuJoCo → Gazebo → Isaac)
검증 하네스는 기존 **`cho_task_manager` controller_check 트리** 사용 (사용자 지정).
`controller_check_torque`가 우리가 개선한 토크 제어기 전부를 exercise:
joint_space_impedance / joint_space_qp / task_space_qp / task_space_impedance /
operational_space (+ gravity_compensation hold). 각 제어기를 3초 이동(B→A, Down/Up)으로 구동.
base controller가 `/log/joint_pos`(desired vs current), `/log/ee_pose`(ref/des/curr) 퍼블리시.
플롯: `python/plot_joint_pos_log.py`, `python/plot_pose_log.py`.

검증 방법(환경별):
1. `ros2 launch cho_bringup_franka bringup_<env>_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller` (필요 모드별)
2. `ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_<mode> use_sim_time:=true`
3. `/log/joint_pos`·`/log/ee_pose` 기록 → **수정 전(git stash)/후** desired-current RMS·peak 추적오차 비교.
   (deterministic 보조 검증: 트래젝토리 pos를 수치미분해 vel/acc가 해석해와 일치 + 경계 0 확인.)

- [x] MuJoCo(1kHz): **9개 제어기 전부** 실측 (torque 5 + position 2 + velocity 2). 아래 A/B 정량표.
      joint RMS −56%, task 회전 RMS −52~65% 확인.
- [x] Gazebo(1kHz): **7개 제어기** 실측 (torque 5 + position 2). velocity는 vendored
      franka_ign 플러그인 미지원(README) → 수집 제외. 좀비 프로세스 오염 정리 후 재수집:
      position 0.0015 rad, task_ik 0.2mm. task_space_impedance만 시작자세 의존 10~40mm(게인 튜닝건).
- [x] Isaac(250Hz): **9개 제어기** 실측. 자체 ROS_DOMAIN_ID(79). velocity/IK 우수(관절 0.0013 rad,
      태스크 1.7mm), torque/position은 250Hz 보수적 게인 특성으로 오차 큼(발산 아님, README).
- [x] 회귀 없음: idle 로깅 clean(Phase 5), 전 goal 성공 판정 유지, 수렴오차 불변.
- **검증 리포트(3환경 종합)**: `scratchpad/wp_report.html` →
      https://claude.ai/code/artifact/95127a4a-789c-40d0-bf96-3a39485130ee

### MuJoCo 검증 결과 (joint_space_impedance, A→B 1.5s, 동일 시작점, 전이 제외)
| 지표 | ff OFF(기존) | ff ON(신규) | 개선 |
| --- | --- | --- | --- |
| RMS 전체 | 0.00491 | **0.00215** | **−56%** |
| PEAK 전체 | 0.01470 | **0.00981** | −33% |
| j2 rms / j6 rms | 0.0089 / 0.0091 | **0.0035 / 0.0040** | −61% / −56% |
| 최종 수렴오차 | 0.00255 | 0.00237 | 동일(양쪽 정상) |
→ **모션 중 추적 RMS 절반, 수렴은 그대로.** 목표 달성 확인. sim 정리 완료, 트리=ff-on.
측정 하네스: `scratchpad/track_test.py`(goal 전송+`/log/joint_pos` 기록+오차계산).

### MuJoCo 태스크공간 A/B (상대 20mm + 45°회전, 2s, 회전 위주 → SE3 각속도 ff 검증)
측정 하네스: `scratchpad/track_task.py`(TaskSpace goal + `/log/ee_pose`).
| 제어기 | 회전 RMS OFF→ON | 병진 RMS OFF→ON | 수렴 |
| --- | --- | --- | --- |
| task_space_qp (SE3, P1) | 2.49° → **1.19°** (−52%) | 4.23 → 4.08 mm | 동일 |
| task_space_impedance (P2) | 2.82° → **0.99°** (−65%) | 2.99 → 2.73 mm | 동일(<0.2mm) |
| operational_space (P2) | 0.167° → **0.077°** (−54%) | 1.33 → **0.73 mm** (−45%) | 동일 |
→ **가장 까다로운 SE3 각속도 프레임 규약이 실측 검증됨** (회전 추적 −52~65%). 세 제어기
모두 goal 성공·안정, 수렴 불변. 트리=ff-on 복원, 빌드 클린, stash 없음.

### 검증 중 발견한 부수 이슈
- 원인 확정: **`ee_state_broadcaster`** (상시 active, `FrankaBaseController::update()` 호출하지만
  `q_arm_des`/`H_ee_des` 미설정 → 초기화 안 된 값 = 실행마다 0(MuJoCo) 또는 garbage(Gazebo)를
  `/log/joint_pos`·`/log/ee_pose`에 publish). switchable arm 컨트롤러는 하나만 active라 정상.
  `gravity_compensation`도 desired 미설정.

## Phase 5 — 로깅 오염 수정 (게이팅 패치, 사용자 확정) — 완료
- [x] base에 `should_publish_arm_log()` 가상함수(기본 true) + base `update()`에서 log 게이팅.
- [x] `ee_state_broadcaster`에서 `false` 오버라이드.
- [x] base `on_activate`에서 `q_arm_des/H_ee_des/H_ee_ref = 측정값` 초기화(gravity_comp 등 방어).
- [x] 빌드 성공 + MuJoCo 재검증: idle 4s 감사 결과 `/log/joint_pos`·`/log/ee_pose` 각 ~2000샘플
      **zero=0, garbage=0, ok=100%** (수정 전엔 절반이 0/garbage). `/ee_state/pose`는 무영향 확인.
  → 기존 토픽·plot 스크립트 유지하는 최소 패치. **Phase 5 완료.**
  (참고: publisher COUNT는 여전히 7 — 모든 파생 컨트롤러가 on_configure에서 publisher 객체를
   생성하기 때문. 실제 publish 데이터는 clean. 객체 수까지 줄이려면 네임스페이스 재설계 항목에서.)

## Phase 4 — 확장 & 공식화 (사용자 확정: 4개 전부 진행, 2026-09-01)
남은 항목 전부 착수. 스코핑(Explore subagent 병렬) → 구현(subagent) → 리뷰(감독 subagent) → 빌드/검증 순.

### WS-1 — OpenArm / UR 확장 + 검증 ✅ 완료
- [x] Phase 5 로깅 게이팅을 `OpenArmBaseController`에 이식(virtual `should_publish_arm_log`,
      update() 게이팅, `EEStateBroadcaster` override=false, on_activate에 누락됐던 `q_arm_des=q_arm` 시드 추가). 빌드 성공.
- [x] **UR는 작업 불필요** 확인: 항상-active인 base 파생 컨트롤러가 없음(브로드캐스터 부재, Gripper는 base 미상속) → 오염원 없음.
- [x] Phase 1 ff가 OpenArm impedance에 반영됨 확인(`joint_space_impedance_controller.cpp:168`이 `sample.vel` 소비).
- [x] **MuJoCo 검증**(도메인 55 격리): idle `/log/joint_pos`·`/log/ee_pose` 각 ~1.8k샘플 **100% ok**(zero/garbage=0,
      게이팅 작동). impedance 추적 peak 0.0119 / **rms 0.0027 rad**(Franka 동급). Isaac은 GPU(INTACT 학습) 점유로 보류 —
      게이팅은 환경무관 base 코드라 MuJoCo로 입증됨.

### WS-2 — 코드 정리 (저위험) ✅ 완료
- [x] Timeopt `computeNext` `-Wreturn-type` UB: 두 함수의 `else{assert(false);}` 뒤에 `return m_sample;` 추가.
      빌드 로그에서 `-Wreturn-type` 경고 2건 **사라짐 확인**.
- [x] `task_space_impedance_controller.cpp` 미사용 `R_spatial`(95-97) 제거 + `trajectory_se3.cpp` Timeopt 죽은 locals(249-252) 제거.
- (부수 발견, 범위 밖) `trajectory_euclidian.cpp` `assert(m_size = size())` (`=` vs `==`) 및 setMaxAccel/Vel 스왑 의심 — Timeopt 경로, 별도 검토.

### WS-3 — 환경별 게인 divergence 정리 (로드맵 5번) ✅ 완료
- [x] real/gazebo/mujoco/isaac 게인 비교표 작성. 정당 divergence(중력 ff 3분기 regime: real=Coriolis만/gazebo=+hand/mujoco·isaac=full;
      Isaac 250Hz sqrt(kp)·τ<0.4) vs 우발 drift 구분.
- [x] gazebo/mujoco config에 regime 헤더 주석 추가, isaac task_space_qp의 낡은 "Left at MuJoCo values" 주석 수정. **숫자 무변경, real/ 미접촉.**
- [x] **잠재버그 2건 수정 + MuJoCo A/B 재검증**: joint_space_qp kd 40→126.5(=2√4000): RMS 0.0005 rad(=기존), peak −26%.
      operational_space kd_r 33.9→169.7(=2√7200): 회전 RMS −10%, 병진 무변화. 회귀 없음 → keep.

### WS-4 — 로깅 네임스페이스 재설계 (사용자 확정: **표준안 control_msgs 채택**) — 진행 중
설계(스코핑 확정): 관절 로그 → per-controller `~/controller_state` = `control_msgs/msg/JointTrajectoryControllerState`
(reference=desired, feedback=current, pos/vel[/acc]); 카테시안 로그 → per-controller `~/ee_state` = `cho_interfaces/PoseLog`
(표준 타입에 SE3 3-pose 없음 → 커스텀 유지, Header 추가). 네임스페이스화로 게이팅 워크어라운드 불필요해짐.
- [x] **Phase A(가산, 비파괴)**: 3개 base에 `~/controller_state`+`~/ee_state` 추가(구 `/log/*` 유지), `control_msgs` 의존성 추가.
      빌드 4패키지 클린. **스모크 통과**: active만 ~496Hz 발행, inactive/broadcaster 무발행(게이팅 확인), reference/feedback 정상.
      **독립 리뷰: CLEAN**(RT 안전·preallocation·필드매핑·게이팅 전부 정확, 확정 버그 0).
- [x] **Phase B(소비자 이관)**: `plot_joint_pos_log.py`(bag 타입 자동감지 → JointLog/JointTrajectoryControllerState 둘 다 처리 + `--topic`),
      `plot_pose_log.py`(`--topic`), README Log 섹션·CLAUDE.md(낡은 `/log/joint_states` + JointLog 참조) 갱신.
- [x] **Phase C(구 제거)**: franka/openarm/ur 3개 base에서 `/log/*` publisher·publish 블록·JointLog include 제거,
      `cho_interfaces` 재빌드. **5패키지 빌드 성공.** `should_publish_arm_log` 게이팅은 유지(신규 토픽에 재활용).
      **최종 스모크**: 구 `/log/*` 완전 소멸, 신규 컨트롤러별 토픽만 active가 ~477Hz 발행 ✓.
- [!] **JointLog.msg는 유지** — 4번째 패키지 `cho_controller_fr5`(사용자 진행 중 작업)가 아직 구 `/log/joint_pos`+JointLog 사용.
      FR5는 **범위 밖 + 동시 실행 중**이라 미접촉. FR5까지 마이그레이션해야 JointLog 폐기 가능(후속 항목, 사용자 확인 필요).
- 블라스트 반경 ~18파일(FR5 제외). 라이브 구독자 없음(소비자는 오프라인 plot 2개뿐).

---

## 검증 지표
- **추적**: 구간 중 |desired − current| RMS / peak 오차 감소 (특히 impedance/qp/task_qp).
- **수렴**: goal 종료 후 정상상태 오차가 success_threshold 이내, 오버슈트 없음.
- **안정성**: 정지 시 관절 떨림 없음(README: rest 0.0006 rad), smoke 전 모드 통과.

## 리스크 & 롤백
- SE3 각속도 프레임 오류 위험 → 방향 고정 모션(R_init=R_goal)에선 ff=0이라 무영향; 프레임은
  `frameVelocity=LOCAL`, `TaskSE3Equality`의 `m_wMl.actInv` 규약으로 검증 완료.
- acc 피드포워드는 물리적으로 작음(예: 0.3m/4s → 0.11 m/s²) → QP 불안정 유발 안 함.
- 각 Phase는 독립 커밋 단위. 문제 시 `cho_controller_common` 트래젝토리 diff만 되돌리면 원복.
