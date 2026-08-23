# cho 세션 로그

컨텍스트가 끊긴 뒤에도 이어갈 수 있도록 남기는 인수인계 문서.
좌표·실행법·고유 규칙은 `CLAUDE.md` 에 있다 (자동 로드됨).
**이 파일은 "무엇을 왜 결정했고 무엇이 열려 있나"** 만 담는다.

---

## 0. 지금 상태

<!--
  ★ 이 섹션이 이 파일의 전부다. 15줄 안에서 끝낼 것. 덮어쓰고, 누적하지 않는다.
  새 세션이 이것만 읽고 30초 안에 따라잡을 수 있어야 한다.

  - 지금의 핵심 상태 한 문장 (굵게)
  - 왜 그런가 — 원인과 근거 수치
  - 지금 돌고 있는 것 / 예측
  - 열린 질문은 어느 섹션인가 (§ 번호로)

  넣지 말 것: 과정, 버린 접근, 코드 조각, 이미 닫힌 것, "~할 예정" 류의 계획
-->

**OpenArm: 단일·양팔 × physx/newton × torque/position/velocity 12 조합 전부 동작.**
**같은 궤적-시계 버그가 Franka 에도 있어서 같이 고쳤다. UR 은 무관.**

**position 이 안 되던 진짜 원인은 게인이 아니라 궤적 시계였다.** 컨트롤러가
`1/get_update_rate()` 로 자기 시계를 돌리는데 그 함수는 컨트롤러가 CM 레이트를 상속하면
(이 프로젝트 전부) **0** 을 반환하고, 폴백이 1 kHz 하드코딩이었다. MuJoCo(1 kHz)에선 우연히
맞았고 Isaac(250 Hz)에선 시계가 sim time 의 1/4 속도 → **모든 goal 이 4 배로 늘어졌다.**
0.02 rad/s 로 기어가는 모습이 "드라이브가 약해 중력에 처진다" 와 똑같이 보였다.
`nominal_period()` 가 측정 주기로 레이트를 추정하도록 고쳤다. velocity 컨트롤러도 같은 버그.

stiffness 도 이 팔의 중력 부하에서 재산정(k = max|τ_g| / 1.89 mrad, d = 2√(k·M_max)).
기존 MuJoCo 유래 값은 최악 포즈 norm 0.0198 로 임계값 초과였다 → 0.0045.

토크 진동 건은 **지연 마진**(DDS 왕복 20~25 ms → wn=15 안정, wn=18 부터 리밋사이클).
Newton 은 **variant 선택이 결정적** — `mujoco` 엔 DriveAPI 가 없어 액추에이터가 안 깔린다.

열린 것은 **`todo/OPENARM_TODO.md`** 하나뿐 (§1 안의 "남은 것" 은 그 시점의 기록이다).

---

## 1. 기록

<!--
  세션별 상세를 append. `### YYYY-MM-DD` 헤더.
  200줄 넘으면 docs/sessions/<YYYY-MM-DD>.md 로 밀어내고 여기엔 한 줄 색인만 남긴다:
    - 2026-08-03~05 dense 스케일 조사, α 지형 문제 발견 → docs/sessions/2026-08-05.md
-->

### 2026-08-23

OpenArm Isaac 토크 제어가 수렴하지 않고 진동한다는 보고 → 원인 규명 + 수정.

**진단.** MuJoCo 는 정상(최악 오차 0.0006 rad, 진폭 1e-5)인데 Isaac 만 8.4 Hz 로
지속 진동, 전 관절이 같은 주파수 → 관절별 미스튜닝이 아니라 루프 레벨 불안정.
게인 스윕(wn=12/15/18/22/26/30)으로 경계가 15와 18 사이임을 확인:

| wn | 결과 |
|----|------|
| 12 | 안정, 잔류 0.021 rad p-p |
| 15 | 안정, 잔류 0.014 rad p-p |
| 18 | **리밋사이클 1.3 rad p-p @ 4 Hz** |
| 30 | **리밋사이클 2.7 rad p-p** (기존 설정값) |

관측 진동수로 역산한 유효 지연 ≈ 24 ms (250 Hz 에서 약 6 스텝). PD 루프의 이득
교차점이 62 rad/s 라 위상 여유가 음수가 된다.

**두 번째 문제.** wn 을 낮추면 안정하지만 PhysX 관절 마찰이 Coulomb 이라 오차가
점근적으로만 줄어든다(norm 0.107 → 0.060 → 0.028, 8 초 걸림). 액션 서버 성공 판정은
7관절 오차 **2-norm < 0.05**(관절별이 아님)이고 데드라인이 duration+2 s 라 전부 ABORT.
비례 게인으로는 못 고치는 문제 → 적분항 추가.

**수정.**
1. `JointSpaceImpedanceController` 에 `ki_joint` / `integral_clamp` 추가.
   τ = M·(kp·e + kd·ė + ki·∫e) + nle. 토크가 클립된 관절은 그 사이클 적분을
   롤백하는 anti-windup. 파라미터 미설정이면 완전히 꺼짐(MuJoCo 무영향).
   ki_joint_ 를 configure 때 0 으로 만들어 RT 루프에서 분기 없이 항상 더한다.
2. `config/isaac/controllers.yaml`, `controllers_bimanual.yaml`:
   kp 900→225, kd 60→30, ki 1350, integral_clamp 0.05.

**검증** (도메인 62, 학습 프로세스 13개 동시 구동 중).
단일 팔 4 목표(뷰포트 열고 닫고, duration 2 s 공격적 스텝 포함) 전부 SUCCEEDED,
최악 오차 0.0095~0.0269 rad. 양팔 4 목표 전부 SUCCEEDED, 최악 0.0011~0.0084 rad.
좌우 값이 정확히 부호 대칭(0.3993/-0.3993)인 것까지 확인. **joint7 ±0.34 → 0.0000.**
MuJoCo 회귀 0.0008 rad(변화 없음), `cho_description_openarm` 53 테스트 통과.

`colcon test` 가 워크스페이스 전체에서 159 failures 를 보고하는데 이는 사용자 환경의
`anyio` pytest 플러그인이 깨진 것(`_pytest.scope` 없음)으로 이 변경과 무관하다.
`pytest -p no:anyio` 로 돌리면 통과한다.

**남은 것**
- Newton 물리 백엔드 (`run_isaac_sim.py` 에 `--physics-engine newton`. 관절이
  `IsaacJointAPI` 만 갖고 있어 `_author_drive_gains` 실패, `_author_armatures` 무동작).
- 양팔 task_manager 배선(`openarm_bimanual.yaml` + 트리) 미착수.
- 실물 CAN(M5) 미검증.

#### Newton 물리엔진 추가 (같은 날)

`--physics-engine newton` 을 실제로 동작시켰다. 막혀 있던 지점은 세 개였고 전부 별개다.

**1) 엔진을 바꿔도 USD variant 는 안 바뀐다.** 에셋 루트에 `Physics` variant set 이 있고
(`physics` / `physx` / `mujoco` / `none`), Newton 은 `mujoco` 를 읽는다 — 근거는 Newton 자신의
테스트 헬퍼 `isaacsim.physics.newton/tests/test_rigid_body.py` 의
`{"physx": "physx", "mujoco": "newton"}` 매핑. `auto_switch_on_startup` 은 솔버만 등록하고
variant 는 건드리지 않아서 명시적으로 선택해야 한다. 그리고 `mujoco` variant 아래에서는
관절이 `IsaacJointAPI` 만 갖는다 — DriveAPI 도 PhysxJointAPI 도 없다. 그래서 PhysX 용
USD 오서링 3종(`_author_articulation_solver` / `_author_armatures` / `_author_drive_gains`)은
전부 physx 전용으로 묶고, Newton 은 play 후 텐서 API 로 넣는다.
`set_dof_armatures` / `set_dof_gains` 는 Newton 백엔드에선 정상 동작한다 (PhysX 백엔드에서
`set_dof_gains` 가 관절을 1e3 rad 로 날려버리는 6.0.1 버그가 physx 경로가 USD 를 쓰는 이유였다).
armature 는 필수다 — Newton 기본값이 전 관절 0.1 kg·m² 라서 j1/j2(0.0081) 는 12 배 틀린다.

**2) 충돌 메시 스케일이 음수였다.** 임포터가 메시 handedness 를 `(0.001, -0.001, 0.001)` 로
표현하는데, PhysX 는 음의 행렬식을 받아주지만 Newton 의 MuJoCo 솔버는 전부 음수로 분해한 뒤
`Only plane shapes are allowed to have a size of zero` 로 죽는다. 로그에는
`[Newton] Initialization failed` 로만 뜬다. 7 개 충돌 스케일을 절댓값으로 뒤집는다
(=hull 이 미러링되지만 self-collision 이 꺼져 있고 접촉 상대가 지면뿐이라 무해). 시각 메시는 안 건드린다.

**3) 프로파일 값 중 두 개는 Newton 에 못 넣는다.** `joint_friction` 은
`set_dof_friction_properties` 가 이 릴리스에서 로그만 찍는 스텁이라 적용 불가.
effort limit 도 없다 — DOF 당 1e6 Nm 를 보고하므로 컨트롤러의 `clip_torque`(URDF 한계)가
유일한 상한이다. 둘 다 로그로 명시한다.

**측정** (도메인 62, 학습 프로세스 19개 동시 구동).

| | physx | newton |
|---|---|---|
| 단일 팔 4 목표 최악 오차 | 0.0059~0.0269 rad | **0.0014~0.0072 rad** |
| 양팔 4 목표 최악 오차 | 0.0011~0.0084 rad | **0.0002~0.0004 rad** |
| 24 초 홀드 | 0.006~0.027 | **0.0000 rad** |

Newton 이 더 정확한 건 튜닝이 아니라 Coulomb 마찰이 없어서다 — 적분항이 갈아낼 정상편차가
아예 없다. 게인은 두 엔진이 공유한다(wn 천장을 정하는 건 DDS 왕복이라 엔진과 무관).

**회귀**: physx 경로 변화 없음(A 목표 cross=+3.28 s 로 동일), Franka Isaac 정상 부팅
+ 전 컨트롤러 active. `cho_bringup_isaac` 은 Franka/UR 공용이라 이게 필수 확인이었다.

**position 모드는 두 엔진 모두 못 쓴다 (기존 문제, 이번 작업과 무관).**
중력 처짐이 임계값 0.015 rad 을 크게 넘는다 — physx 0.060, newton 0.297, 둘 다 중력 부하
관절(1,2,4,6)에서만. 프로파일의 `arm_position_stiffness` 가 MuJoCo 액추에이터 파일에서
그대로 넘어온 값이고 Isaac 드라이브 기준으로 재튜닝된 적이 없다. Franka 가 같은 벽에
부딪혀 공식 franka.usd 값으로 올려 해결했으므로 OpenArm 도 동일 작업이 필요하다.
(Newton 이 5 배 더 나쁜 이유는 effort 포화이 아니다 — 위 1e6 Nm 참조. 드라이브 stiffness
단위 문제로 보이나 확인 안 했다.)

**남은 것**
- position 모드 드라이브 stiffness 재튜닝 (두 엔진 공통, Franka 선례 있음).
- Newton 에서 관절 마찰이 필요해지면 Isaac 릴리스 업데이트를 기다려야 한다.
- 양팔 task_manager 배선(`openarm_bimanual.yaml` + 트리) 미착수.
- 실물 CAN(M5) 미검증.

#### position/velocity 모드 수정 — 원인은 stiffness 가 아니라 궤적 시계 (같은 날)

"position stiffness 재튜닝" 으로 시작했으나 **진단이 틀렸다는 걸 측정으로 잡았다.**

처음엔 Franka(22918)/UR(20000) 대비 OpenArm 값(800/500/120)이 25~170 배 작은 게 원인으로
보였다 — 사용자가 "아예 asset 이 다를 가능성" 을 지적한 게 이 비교의 계기였고, 실제로
우리 값은 Isaac 드라이브 값이 아니라 MuJoCo `actuators_position.xml` 의 kp 였다.

그런데 Pinocchio 로 중력 토크를 계산해보니 **joint1 은 어느 포즈에서도 τ_g = 0** 인데
측정 오차는 0.0398 rad 이었다. 중력 처짐이면 불가능하다. 시계열을 보니 팔이 **멈춘 게 아니라
0.02 rad/s 로 계속 기어가고 있었다** — 수렴 실패가 아니라 궤적이 안 끝난 것.

원인: `joint_space_position_controller` 가 `traj_clock_ += 1/get_update_rate()` 로 자기 시계를
돌리는데, `get_update_rate()` 는 컨트롤러가 CM 레이트를 상속할 때 **0** 을 반환한다
(`update_rate_ = 0` 기본값, 이 프로젝트의 어떤 config 도 컨트롤러별 update_rate 를 안 준다).
폴백이 `0.001` 하드코딩이라 Isaac 250 Hz 에서 시계가 4 배 느리게 갔다. 로그로 확인:
`trajectory clock: get_update_rate()=0 -> dt=0.001000 s` (실제 CM 은 250 Hz).
MuJoCo 가 1 kHz 라 거기선 우연히 정확했고, 그래서 이 버그가 안 드러났다.

수정: `OpenArmBaseController::nominal_period(period)` — get_update_rate() 가 0 이면 측정
주기에서 레이트를 추정(첫 정상 사이클로 시드 후 EMA)한다. 측정 주기를 직접 시계에 더하지
않는 이유는 원 주석대로 지터가 2 배라 위치 명령 스텝이 흔들리기 때문. position/velocity
컨트롤러 둘 다 적용.

**그 다음에야** stiffness 재산정이 의미가 있었다. 다른 로봇 값을 베끼지 않고 이 팔의
중력 부하에서 유도:

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
|---|---|---|---|---|---|---|---|
| max τ_g [Nm] | 0.00 | 11.07 | 4.13 | 4.18 | 0.55 | 0.53 | 0.53 |
| k 신규 | 6000 | 6000 | 2250 | 2250 | 300 | 300 | 300 |
| k 기존 | 800 | 800 | 500 | 500 | 120 | 120 | 120 |
| d 신규 | 103 | 103 | 48 | 48 | 4.4 | 4.3 | 3.5 |

k = max|τ_g| / 1.89 mrad (1.89 = 0.015/√7/3), d = 2√(k·M_max). PhysX·Newton 모두 드라이브를
암시적으로 적분하므로 높은 stiffness 가 이산 안정성을 해치지 않는다.
기존 값은 테스트 포즈는 통과(최악 0.0058)하지만 **최악 포즈 norm 0.0198 로 임계값 초과**,
신규 0.0045.

**측정** (임계값 position 0.015 / velocity 0.02 / torque 0.05, 7관절 오차 norm 기준)

| | physx | newton |
|---|---|---|
| position 표준 포즈 | norm 0.0019 / 최악 0.0013 | **동일** |
| position 최악중력 포즈 | norm 0.0029 / 최악 0.0018 | **동일** |
| torque (회귀) | 0.0062 | 0.0011 |
| velocity | norm 0.0144 통과 | **실패 norm 0.696** |
| MuJoCo (회귀) | torque 0.0007 / position 0.0059 | — |

두 엔진이 position 에서 **완전히 같은 수치**가 나온다는 것은, 앞서 기록했던
"newton 이 position 에서 5 배 나쁘다" 가 시계 버그 + Newton 이 실시간보다 느려 같은 벽시계
동안 sim 초가 적게 기록된 것의 합작이었음을 뜻한다. 그 주석은 정정했다.

**Newton velocity 는 동작하지 않는다.** `arm_velocity_damping` 은 들어가는데 드라이브가
속도 타깃을 따라가지 않아 중력에 표류한다 (joint4 가 목표 0.8 을 지나 1.21 까지 가고
joint1 은 아예 안 움직임). physx 를 쓸 것. 프로파일과 런치 인자 설명에 명시.

**남은 것**
- Newton velocity 모드 (드라이브가 joint_target_vel 을 안 따름).
- (해결됨) Franka 도 같은 버그였고 고쳤다 — 아래 별도 항목 참조.
- 양팔 task_manager 배선, 실물 CAN(M5) 미검증.

#### 같은 버그를 Franka 에서도 확인하고 수정 (같은 날)

**UR 은 무관.** `get_update_rate()` 를 아예 호출하지 않고 액션 서버에 실제 `time` 을 그대로
넘긴다(자체 궤적 시계 없음). CM 이 500/250 Hz 인 것과 무관하다.

**Franka 는 동일한 버그였다.** `cho_controller_franka` 의 세 컨트롤러가 같은
`get_update_rate()==0 -> dt=0.001` 폴백을 쓴다:
`joint_space_position_controller`, `joint_space_velocity_controller`,
`task_space_velocity_controller`. 셋 다 franka isaac config 에 선언되어 있고
`POSITION_CONTROLLERS`/`VELOCITY_CONTROLLERS` 목록으로 실제 스폰된다 — 잠재 버그가 아니다.
franka isaac 의 CM 이 250 Hz 라 그대로 물리고, gazebo/mujoco/real 은 1000 Hz 라 폴백이
우연히 정확해서 무영향이었다(그래서 안 드러남).

수정: `FrankaBaseController::nominal_period()` 추가 후 세 곳 교체. OpenArm 과 동일한 구현.

**수정 전후 (Franka Isaac, 4 s goal)**

| | 수정 전 | 수정 후 |
|---|---|---|
| position | ABORT, 4 s goal 이 **17.0 s** (4.25 배) | **SUCCEEDED**, norm 0.0015 / 최악 0.0011 rad |
| velocity | ABORT, norm 이 14.2 s 에야 임계값 통과 | **SUCCEEDED**, norm 0.0030 / 최악 0.0026 rad |
| task_space_velocity | (미측정) | **SUCCEEDED**, 요청 0.100 m 중 0.0988 m 이동 |

**회귀**: Franka Isaac torque norm 0.0060(변화 없음), 컨트롤러 10 개 전부 로드.
Franka MuJoCo(1 kHz) position 0.0066 / velocity 0.0033 — 폴백이 원래 맞던 환경도 그대로.

부수 발견: `cho_bringup_franka/config/isaac/controllers.yaml` 은 최상위 `/**:` 키가 16 번
반복된다. ROS 2 파라미터 로더는 병합하므로 런타임은 정상이지만 PyYAML 로 읽으면 마지막
블록만 남는다 — 이 파일을 스크립트로 파싱할 때 주의.

#### 양팔 position/velocity 모드 추가 + velocity 게인 (같은 날)

양팔은 torque 컨트롤러 블록만 있어서 position/velocity 로 브링업하면 스폰할 게 없었다.
`config/{mujoco,isaac}/controllers_bimanual.yaml` 에 팔당 2 개씩 4 개 블록 추가
(`left_/right_joint_space_{position,velocity}_controller`) + controller_manager 에 타입 등록.
런치 쪽은 손댈 게 없었다 — `launch_utils.per_arm()` 이 이미 프리픽스를 붙인다.
MJCF 도 `xml/openarm_v10_bimanual/scene_{position,velocity}.xml` 이 이미 있었다.

**velocity 게인 kp_joint 10 → 30** (단일·양팔, MuJoCo·Isaac 전부). 이건 강성이 아니라
레퍼런스 추종 게인이라 정상상태 잔차가 `tau_g/(kv*kp)` 로 kp 에 반비례한다. 10 에서는
단일 팔 norm 0.0165(임계값 0.02, 82%)로 아슬아슬했고 **양팔은 0.0246 으로 ABORT** 였다 —
양팔은 토르소에 붙어 팔이 다른 각도로 매달려 중력 토크가 더 크다. 30 으로 정확히 1/3 이 됐다.

**측정** (오차 norm, position 임계 0.015 / velocity 0.02)

| | 단일 | 양팔(좌/우) |
|---|---|---|
| MuJoCo position | 0.0086 | 0.0127 |
| MuJoCo velocity | 0.0055 (기존 0.0165) | 0.0083 (기존 0.0246 ABORT) |
| Isaac physx position | 0.0019 | 0.0027 |
| Isaac newton position | 0.0019 | 0.0028 |
| Isaac physx velocity | 0.0144 | 0.0108 |
| Isaac newton velocity | **실패** | **실패 (1.06)** |

(Newton velocity 는 이 시점엔 실패했고, 아래 항목에서 해결했다.)

#### Newton velocity 수정 — 원인은 Physics variant 선택 (같은 날)

Newton 에서 velocity 만 안 되던 이유를 찾았다. **게인이 아니라 액추에이터가 아예 없었다.**

Newton 모델을 직접 파싱해 `joint_target_mode` 를 찍어보니 전 DOF 가 **`NONE`** 이었다.
`JointTargetMode.from_gains()` (newton/_src/sim/enums.py) 는 `has_drive=False` 면 무조건
NONE 을 반환하고, 우리가 쓰던 `mujoco` variant 는 관절에 `UsdPhysics.DriveAPI` 가 없다.
NONE 이면 position/velocity 액추에이터가 설치되지 않아 `joint_target_pos/vel` 이 그냥 무시된다.

- torque 는 effort 를 `joint_f` 에 직접 넣어 액추에이터가 필요 없다 → 멀쩡히 동작
- position 도 통과했었다 → 그래서 아무도 이상하다고 못 느꼈다
- velocity 만 드러났다: 중력 없는 joint1 이 **전혀** 안 움직이는 게 단서였다

**수정: Newton 의 Physics variant 를 `mujoco` → `physics` 로.** `physics` 에는 DriveAPI 가
있어서, play 전에 게인을 authoring 하면(PhysX 경로가 쓰던 `_author_drive_gains()` 그대로)
임포터가 DOF 별로 올바른 액추에이터를 설치한다. 프로브로 확인:

| authoring | 결과 모드 | ke / kd |
|---|---|---|
| k=6000, d=103 | POSITION | 6000 / 103 |
| k=0, d=25 | VELOCITY | 0 / 25 |
| k=0, d=0 | EFFORT | 0 / 0 |

덤으로 **per-degree 변환이 두 엔진 동일**하다는 것도 확인했다 — USD 에 25*π/180 을 쓰면
`joint_target_kd = 25.0` (per rad) 로 들어온다. 즉 `_author_drive_gains()` 의 π/180 은
Newton 에도 그대로 맞고, 이제 두 엔진이 **같은 코드로 게인을 authoring** 한다.
런타임 텐서 API 는 armature 전용으로 축소(‘newton:armature’ 는 deprecated, physx 것은 무시).

`mujoco` 가 Newton 의 기본 선택(auto_switch)이라는 게 함정이었다. 엔진을 바꿔도 variant 는
안 바뀌므로 어느 쪽이든 명시 선택이 필요하다.

**측정** (오차 norm; position 0.015 / velocity 0.02 / torque 0.05)

| | physx 단일 | physx 양팔 | newton 단일 | newton 양팔 |
|---|---|---|---|---|
| torque | 0.0049 | ✓ | 0.0002 | 0.0004 |
| position | 0.0023 | 0.0027 | 0.0022 | 0.0028 |
| velocity | 0.0057 | 0.0108 | **0.0057** | **0.0108** |

velocity 가 두 엔진에서 같은 수치인 건 이제 같은 authoring 게인을 읽기 때문이다.
**12 조합 전부 SUCCEEDED.**

이후 남은 작업은 전부 `todo/OPENARM_TODO.md` 로 옮겼다. 문서도 채웠다:
README(브링업+액션+엔진 비교), CLAUDE.md(패키지 맵 + nominal_period 규칙),
docs/tasks.md(OpenArm task), docs/installation.md(실물 미지원 명시).

부수: 액추에이터 모드를 런타임에 읽어 로그하려 했으나 `_physics_articulation_view` 래퍼가
릴리스마다 달라 private 경로가 안 맞았다. 우리가 authoring 한 게인에서 같은 규칙으로
유도해 찍도록 바꿨다(`newton arm drive : VELOCITY` 형태). 게인이 둘 다 0 이면 경고한다 —
이 버그를 즉시 잡아줄 체크다.

### 2026-08-22

Isaac Sim 6.0.1 + ROS 2 Humble 통합. 계획: `~/.claude/plans/isaacsim-lazy-cray.md`.

**결정한 구조** — ROS 쪽 controller_manager 에 하드웨어 플러그인만
`topic_based_ros2_control/TopicBasedSystem` 을 물리고, Isaac 쪽은 OmniGraph
ROS2 bridge 노드로 `/isaac_joint_states` `/isaac_joint_commands`
`/isaac_gripper_commands` `/clock` 을 주고받는다. 새로 짠 C++ 은 없다.
CM 노드는 `mujoco_ros2_control/ros2_control_node` 를 재사용한다 — 그 소스에
mujoco 참조가 0건이고 upstream 노드에 `wait_until_started()` + sim-clock 페이싱만
더한 포크라서, sim-clock CM 이 필요한 이 환경에 그대로 맞는다.

**팔/핸드를 별도 하드웨어 컴포넌트로 나눈 이유** — `TopicBasedSystem::write()` 는
관절이 *선언한* command interface 만 `push_back` 한다. 팔(effort 7개)과
핸드(position 1개)를 한 컴포넌트에 넣으면 `name` 8개인데 `effort` 7개 / `position` 1개가
되어 인덱스가 어긋나고, Isaac 이 `position[0]` 을 `fr3_joint1` 에 적용한다.
`read()` 는 이름 매칭이라 두 컴포넌트가 `/isaac_joint_states` 하나를 공유해도 안전하다.

**막혔던 것들과 원인** (전부 재현·확인함):

1. **`*_sc` 링크가 물리를 발산시킴.** URDF 의 자기충돌 보조 링크 9개는 `<inertial>` 이
   없어서 임포터가 질량 0 강체 + fixed joint 로 만든다. 그러면 관절 드라이브
   stiffness 를 80 Nm/rad 만 줘도 첫 스텝에 발산한다
   (`Illegal BroadPhaseUpdateData - non-finite bounds`). 컨버터에 `--strip-links`
   (기본 `.*_sc$`) 를 넣어 제거 → 링크 13개, 안정. `--merge-fixed-joints` 로도
   안정해지지만 그러면 `bota_ft_sensor_wrench` / `fr3_hand_tcp` 가 사라져 FT 를 못 읽는다.

2. **USD 각도 드라이브 게인은 degree 단위.** USD 에 stiffness 400 을 쓰면 물리 뷰는
   22918 (= 400 × 180/π) 로 읽는다. `π/180` 을 곱해서 써야 한다. 이걸 모르면
   57.3배 과도한 게인으로 무조건 발산한다.

3. **`Articulation.set_dof_gains()` 가 상태를 깨뜨린다** (Isaac 6.0.1). 호출 그 자체로
   관절값이 ~1e3 rad 로 튄다 — `dof_indices` 유무와 무관. 그래서 게인은 play 전에
   USD `UsdPhysics.DriveAPI` 로 쓴다. `set_dof_positions` / `set_dof_velocities` 는 정상.

4. **Isaac 프로세스 안에서 rclpy 사용 불가.** 브리지가 시스템 ROS 라이브러리를 쓰려면
   `/opt/ros/humble/setup.bash` 를 source 해야 하는데, 그러면 시스템 rclpy(3.10) 는
   Kit(3.12) 에서 `No module named 'rclpy._rclpy_pybind11'` 로 죽고, Isaac 번들
   rclpy 는 `rclpy.init()` 까지 통과한 뒤 `Node()` 에서 **세그폴트**한다(ABI 불일치).
   Isaac 자체 로그도 "Could not import rclpy" 를 남긴다. → 게이트는
   `ROS2SubscribeJointState` 출력 어트리뷰트 폴링으로, FT 는 범용 `ROS2Publisher` 로
   해결했고, 진짜 ROS 파이썬이 필요한 부분(wrench 스탬핑, tare 서비스)은
   `scripts/isaac_ft_sensor.py` 로 ROS 쪽에 뒀다.

5. **`OnPhysicsStep` 은 on-demand 그래프에서만 발화한다.** `og.Controller.edit` 에
   `pipeline_stage: GRAPH_PIPELINE_STAGE_ONDEMAND` 를 줘야 한다. 안 그러면 조용히
   아무것도 안 나온다.

6. **articulation 루트가 `/fr3` 가 아니라 `/fr3/Geometry/base`.** OG 노드는 경로
   문자열을 그대로 받으므로 `robot.paths[0]` 로 해석해서 넘겨야 한다.

7. **스포너를 CM 시작 시점에 띄우면 안 된다.** Isaac 부팅에 ~40초가 걸리는데 그동안
   `/clock` 이 없어 CM 의 RT 루프가 `wait_until_started()` 에서 멈춰 있고, 스위치가
   자체 5초 타임아웃으로 죽는다(`--controller-manager-timeout` 은 서비스 대기용이라
   무관). 런치에서 `OnProcessIO` 로 Isaac 의 `running:` 마커를 보고 띄운다.

8. **범용 `ROS2Publisher` 의 메시지 타입은 그래프 생성 시 SET_VALUES 로** 줘야
   `inputs:force:x` 같은 동적 어트리뷰트가 생긴다. 나중에 `.set()` 하면 조용히 실패한다.

**게이트가 필요한 이유** — `TopicBasedSystem` 은 command 버퍼를 0 으로 초기화하고
하드웨어 컴포넌트가 active 되는 순간부터 컨트롤러 유무와 무관하게 발행한다. 토크
모드면 팔이 떨어지고 position 모드면 q=0 으로 끌려간다. 그래서 게이트가 열릴 때까지
Isaac 이 홈 자세를 **운동학적으로** 고정한다(게인이 0 이라 드라이브로는 못 잡는다).

**토크 모드에서 드라이브 게인을 건드리지 않는 이유** — 임포터가 URDF 의
`<dynamics damping>` 을 그대로 넣어준다. 이걸 0 으로 덮으면 널스페이스가 완전히
무감쇠가 되어 joint7 이 6초에 0.11 rad 드리프트한다. 안 건드리면 0.02 rad.

**2026-08-22 후반: 사용자가 보고한 joint7 진동 + reach 무반응 추적**

증상 두 개를 각각 계측해서 원인을 분리했다. 모두 재현 후 수치로 확인.

1. **joint7 진동 = armature 누락.** URDF 에는 로터 관성을 적을 방법이 없어서 임포트된
   articulation 은 링크 관성만 갖는다. 손목은 로터보다 ~15배 가벼워 √(kp/I) 가 지연
   대비 너무 커진다. MuJoCo 모델 값(joints1-4 0.195, 5-7 0.074, 손가락 0.1)을
   `physxJoint:armature` 로 넣어 해결. 계측: joint7 위치 진폭 0.188 rad → 0.000.

2. **"안 움직임" = 마찰 의미 불일치 + 게인.** URDF `<dynamics friction="0.2">` 는
   Nm 토크인데 임포터가 PhysX `physxJoint:jointFriction`(구속력에 비례하는 **계수**)에
   그대로 복사한다. 하중 큰 팔에서는 막대한 드래그가 되어 Cartesian 목표 5~8 cm 앞에서
   멈춘다. 0.05 로 낮추고 게인을 §위 규칙에 맞춘 뒤 통과.
   계측: 상대 5 cm 이동 달성률 Down 62%/Up 44% → Down 106%/Up 90%.

3. **중력 보상은 정상이었다.** `gravity_compensation_controller` 단독으로 15초 관측 시
   전 관절 변화 0.0000 rad, EE 이동 0.0000 m. Pinocchio 모델과 Isaac 질량이 일치한다.
   (`*_sc` 링크는 질량 0 이라 제거해도 중력 모델에 영향 없음 — 확인함.)

4. **계측 오염 주의.** 다른 agent 가 같은 도메인에서 MuJoCo 를 돌리고 있어 `/clock`
   퍼블리셔가 2개가 된 구간이 있었고, 그 동안의 마찰·게인 실험 결과가 왜곡됐다.
   도메인 60 으로 격리한 뒤 재측정해서 결론을 다시 세웠다.

**공식 에셋에서 가져온 값** (`<assets_root>/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`)
   - `solverPositionIterationCount=32`, `solverVelocityIterationCount=1`,
     `enabledSelfCollisions=False` (임포터는 아무것도 안 써줘서 PhysX 기본값이었다)
   - 이 에셋은 armature/jointFriction 을 아예 안 쓴다. 다만 그건 내부 PD 위치 드라이브
     전제이고, 우리는 지연 있는 외부 토크 제어라 마찰 0 은 오히려 정착을 막는다(측정됨).
   - `maxJointVelocity=124.6` 이 2.17 rad/s 와 일치 → **USD 각도 값은 degree 단위**라는
     것을 공식 에셋이 확인해 준다. 드라이브 stiffness/damping 을 쓸 때 π/180 을 곱해야 한다.

**Isaac 러너를 로봇 프로필 방식으로 일반화 (UR 추가하면서)**

`cho_bringup_isaac` 패키지를 새로 만들고 러너/컨버터/ROS 헬퍼를 모두 그리로 옮겼다.
로봇별 값(관절 이름, 홈, armature, 드라이브 게인, FT 링크)은 `isaac/robots/*.json`
프로필로 빠졌고 러너는 `--robot-profile` 로 읽는다. Franka 와 UR 이 같은 러너를 쓴다.
핸드가 없는 로봇(UR)은 그리퍼 OmniGraph 노드를 아예 안 만든다.

UR 은 position 제어만 쓰고 드라이브가 Isaac 내부에서 물리 주기로 돌아 왕복 지연에
둔감하다. 그래서 MuJoCo 게인이 그대로 통했고, 바꾼 건 update_rate(500→250)와
per-cycle 인 `max_delta_q`(0.02→0.04, 같은 rad/s) 뿐이다.

`ur5e.urdf` 는 Franka 와 같은 방식으로 `hardware` xacro 인자를 받아 MuJoCo/Isaac
블록을 고른다. 기존 UR mujoco 런치가 파일을 문자열 치환으로 읽고 있어서 xacro 처리로
바꿨다(`$(find ...)` 는 xacro 가 동일 경로로 풀어준다 — 확인함).

**VLA 검증 (Isaac)**

세 모드 모두 `vla_controller` 가 로드·활성화되고 목표 수명주기가 완결된다
(goal accepted → ActionChunk 스트림 소비 → `/vla/trigger_success` → Goal Succeeded).
EE 가 명령된 원(중심 0.5,0,0.4 / 반경 0.05)을 실제로 그리는지까지 확인했다.

| 모드 | 명령 대비 진폭 | 비고 |
|---|---|---|
| position | 98% | 드라이브가 Isaac 내부 물리 주기라 손실 거의 없음 |
| torque | 63% | 대역폭 감쇠 (MuJoCo 93%) |
| velocity | 145% | **오버슈트** (MuJoCo 99.8%) — 튜닝 필요 |

velocity 오버슈트는 Isaac 고유다(MuJoCo 대조로 확인). 드라이브 damping 450→2000,
`kp_joint_vel` 20→10 둘 다 시험했으나 개선되지 않아 되돌렸다. 왕복 지연 때문에
원의 반환점에서 속도 명령을 못 따라가고 지나치는 것으로 보인다.

**테스터 주의** — `vla_action_client.py` 는 기본이 `is_relative=True` 인데, 이 상대
청크 패턴은 **어느 시뮬레이터에서도** 눈에 띄는 움직임을 못 만든다(Isaac 0.1 mm,
MuJoCo 1.7 mm / 명령 50 mm). 각 청크가 현재 자세에 재앵커되면서 서브밀리미터
오프셋만 명령하기 때문이다. 검증에는 `is_relative=False` 와
`--ros-args -p use_sim_time:=true` 를 써야 한다.

**남은 것**
- RTF 0.4~0.45 (렌더가 물리와 같은 rate). app dt 를 키우면 PhysX 가 서브스텝을 돌려
  `/clock` 이 버스트로 나가므로 일부러 같게 뒀다. headless 면 조금 낫다.
- `physics_rate` 를 올리면 지연이 줄어 √kp 상한도 올라간다. 500 Hz 를 시험했을 때는
  RTF 가 0.25 로 떨어져 벽시계 기준으로는 더 느렸다.