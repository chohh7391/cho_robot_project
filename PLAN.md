# OpenArm 실제 로봇 및 MIT 제어 통합 계획

## 1. 목표와 확정된 경계

이 계획의 목표는 기존 Cho 프로젝트의 OpenArm description, bringup, controller 구조를 유지하면서,
`openarm_ros2`의 실제 하드웨어 드라이버와 CAN 계층만 재사용하여 position, velocity,
impedance, torque 제어를 실제 로봇과 시뮬레이션에서 같은 MIT 명령 의미로 제공하는 것이다.

확정된 패키지 소유권은 다음과 같다.

- Robot description: `cho_description/cho_description_openarm`
- Launch/config 및 운용 진입점: `cho_bringup/cho_bringup_openarm`
- Controller: `cho_controller/cho_controller_openarm`
- Upstream vendor source: `extern/openarm_ros2`
- Upstream에서 활용할 범위: 실제 OpenArm hardware driver 및 CAN 관련 코드
- Upstream description, bringup, controller는 Cho 패키지를 대체하지 않는다.

이미 real/mock 경로가 `position`, `velocity`, `effort` command interface 세 개를 함께 claim하고,
`kp_scale`과 측정 관절 위치(`q_measured`) 기반 command seeding으로 MIT의 오래된 position target이
갑자기 적용되는 문제를 방지하는 기반을 갖고 있다. 이 동작을 먼저 검증하고 재사용하며, 같은 기능을
새 interface나 controller로 중복 구현하지 않는다.

단, 현재 `kp_scale`, `kd_scale`은 URDF에서 hardware가 초기화될 때 읽는 parameter이므로 실행 중
mode switch 수단이 아니다. 따라서 현 구조만으로 position/damped/direct torque 사이의 runtime gain
전환이 가능하다고 가정하지 않는다. upstream의 runtime gain 지원 여부를 먼저 조사한 뒤 아래 3단계
설계 게이트에서 구현 전략을 확정한다.

## 2. 공통 제어 계약

관절별 MIT 명령의 논리적 계약은 다음 다섯 값으로 정의한다.

```text
q_des, dq_des, kp, kd, tau_ff
```

실제 로봇과 시뮬레이션이 재현할 목표 식은 다음과 같다.

```text
tau_cmd = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
```

이 계약은 controller와 backend 사이의 의미 규약이다. 반드시 다섯 개의 새로운 ROS 2 command
interface를 추가한다는 뜻은 아니다. 우선 기존 `position + velocity + effort` triple-interface,
`kp_scale`, hardware parameter를 조합하는 현재 구조가 다음을 명확하고 안전하게 표현할 수 있는지
평가한다.

- 현재 triple-interface에서 `position=q_des`, `velocity=dq_des`, `effort=tau_ff`로 대응 가능한가
- `kp`, `kd`의 기준값과 runtime scale/override의 소유자가 명확한가
- controller 전환 때 모든 값이 원자적으로 일관된 상태로 바뀌는가
- pure torque에서 position/velocity feedback 항을 확실히 0으로 만들 수 있는가
- controller manager의 interface claim 및 mode switch 규칙과 충돌하지 않는가

평가 결과 기존 방식이 위 조건을 만족하면 유지한다. 만족하지 못하는 항목이 있을 때만 최소 범위의
추가 interface 또는 mode command를 설계하며, 그 결정과 호환성 영향을 문서화한다.

운용 mode의 의미는 다음과 같이 구분한다.

- Direct(pure) torque: `kp=0`, `kd=0`, `tau_ff=tau_user`
- Damped torque: `kp=0`, `kd>0`, `tau_ff=tau_user` (현재 torque 계열의 기본 안전 profile)
- Compensated torque: direct 또는 damped profile에 명시적인 gravity/Coriolis/마찰 보상을 더한 `tau_ff`

Direct torque에서도 activate/switch 시 `q_des=q_measured` seeding은 유지한다. direct mode에서 position
항은 0이지만 이후 gain ramp, fault recovery 또는 다른 mode로 전환될 때 오래된 목표가 즉시 살아나
step torque를 만들지 않게 하기 위해서다.

## 3. 단계별 실행 계획

### 1단계 — Upstream submodule 고정과 빌드 경계 설정

- [x] `extern/openarm_ros2`를 Git submodule로 등록한다.
- [x] 검토한 upstream 특정 commit SHA에 고정하고, 선택 근거와 갱신 절차를 기록한다.
- [x] submodule 내부 package 목록과 의존성을 조사해 hardware/CAN 재사용에 필요한 최소 package를 확정한다.
- [x] `openarm_hardware`가 요구하는 `openarm_can`의 저장소 위치, 버전, transitive dependency 및
      submodule 포함 여부를 조사하고 동일하게 특정 SHA로 재현 가능하게 만든다.
- [x] upstream description, bringup, controller 및 중복 package가 기본 workspace build에 섞이지 않도록
      `COLCON_IGNORE`, 선택 빌드, 또는 명시적 package allowlist 중 한 정책을 정한다.
- [x] 필요한 upstream 소스를 직접 수정하지 않고 사용할 수 있는지 확인한다. 수정이 불가피하면 patch
    유지 전략보다 Cho 쪽 wrapper/adapter/extension을 우선한다. wrapper로 `return_to_zero` 선택화,
      motor MIT mode 확인 또는 runtime gain 제어를 구현할 수 없을 때만 고정 SHA에 대응하는 최소 patch나
    관리 가능한 fork를 허용하며, upstream 반영 가능성·patch 재적용 테스트·소유자를 함께 기록한다.

고정 기준점과 빌드 경계(2026-09-02 조사):

- `enactic/openarm_ros2`의 공식 `main` HEAD
  `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` (`0.9.2-5-g4e837e1`)를
  `extern/openarm_ros2`에 고정했다. 이 checkout에는 `openarm_hardware`, `openarm_bringup`,
  `openarm_bimanual_moveit_config`, metapackage `openarm`이 있으며 실제 hardware 재사용에는
  `openarm_hardware`만 필요하다.
- upstream의 `openarm.repos`는 별도 `enactic/openarm_can` 저장소의 떠 있는 `main`을 가리키므로,
  공식 `main` HEAD이자 release `1.3.4`인
  `a30364622ca939b8c8a741167c317b3e95e38a49`를 `extern/openarm_can`에 별도 submodule로 고정했다.
  `openarm_ros2` 자체에는 `openarm_can` submodule이 포함되어 있지 않다.
- 기본 Cho 소유권을 보존하기 위해 vendor checkout은 수정하지 않고, 허용 package를
  `openarm_can openarm_hardware` 두 개로 제한하는 명시적 선택 빌드 정책을 사용한다. upstream의
  description/bringup/MoveIt/metapackage는 Cho 실행 경로에서 의존하거나 선택하지 않는다.
- 직접 의존성은 `openarm_hardware`의 `hardware_interface`, `pluginlib`, `rclcpp`,
  `rclcpp_lifecycle`, `OpenArmCAN::openarm_can`이다. `openarm_can`은 Linux SocketCAN을 사용하며 CLI
  도구 빌드에 Ubuntu 22.04의 `libcli11-dev`가 추가로 필요하다. upstream `package.xml`에는 CLI11이
  누락되어 있어 `rosdep`만으로 설치되지 않는다. 또한 `openarm_hardware`는 CMake에서
  `rclcpp_lifecycle`를 직접 찾고 링크하지만 자체 `package.xml`에는 이를 선언하지 않는다.
- 이 SHA 조합은 현재 working tree와 격리한 CLI11 2.1.2 공식 Ubuntu package 경로를 사용한 Humble build에서
  `colcon build --symlink-install --packages-select openarm_can openarm_hardware`를 통과했다. upstream
  소스 patch는 없으며, 초기화·갱신·빌드 명령과 정책은 `extern/README.md`에 기록했다. 고정 SHA와
  package allowlist 및 vendor checkout의 tracked/staged/non-ignored untracked source 무변경 상태는
  `tools/build_openarm_vendor.sh`가 실행 전에 검사한다. vendor `.gitignore`에 해당하는 build/package
  산출물은 source 변경으로 취급하지 않는다.
  아직 새 superproject commit을 만들지 않았으므로 새 commit을 clone하는 의미의 clean-checkout 재현 시험은
  수행하지 않았으며, 이는 1단계 완료 조건의 남은 검증 항목이다.

완료 조건:

- `git submodule status`가 재현 가능한 단일 commit을 표시한다.
- 깨끗한 checkout에서 submodule 초기화 후 필요한 package만 빌드된다.
- Cho와 upstream의 description/bringup/controller package가 중복 선택되지 않는다.
- `openarm_can`까지 포함한 dependency와 각 고정 SHA가 문서화되고 clean checkout에서 재현된다.
- upstream patch/fork가 있다면 wrapper로 불가능한 근거와 SHA별 patch 적용 검증이 남는다.

### 2단계 — 기존 Cho 패키지와 실제 hardware 연결

- [ ] `cho_description_openarm`의 joint 이름, 순서, limits, transmissions/ros2_control 정의를
      upstream hardware driver가 기대하는 값과 대조한다.
- [ ] 필요한 hardware plugin 및 CAN parameter만 Cho description/config에 연결한다.
- [ ] `cho_bringup_openarm`에 real single-arm launch 경로를 구성한다.
- [ ] `cho_bringup_openarm`에 real bimanual launch 경로를 구성하고 좌우 CAN interface, namespace,
      controller manager 및 joint prefix 충돌을 제거한다.
- [ ] mock/sim/real 선택이 기존 bringup 인자 체계를 깨지 않도록 통합한다.

완료 조건:

- single 및 bimanual launch가 Cho description과 Cho controller 설정만 사용한다.
- hardware plugin 로딩과 state/command interface export가 기대한 관절 수와 순서로 일치한다.
- 실제 CAN 장치 없이 수행 가능한 launch/config 검증이 통과한다.

### 3단계 — MIT producer/consumer 설계 게이트와 기존 triple-interface 검증

설계 초안은 `docs/openarm_mit_contract_v1.md`와
`cho_description_openarm/config/mit_command_v1.yaml`에 기록했다. **수학과 interface shape만 승인**됐고
lifecycle gate는 미완료다. 5개 joint double만으로 atomicity/freshness를 보장할 수 없어 arm-scoped
generation/commit/lease/ack/status protocol을 추가 설계했으며, real은 vendor patch가 아니라
`openarm_can`을 composition하는 Cho-owned SystemInterface adapter를 우선한다. 아래 체크는 조사/초안
결정 완료이며 backend API 동결이나 4단계 착수를 뜻하지 않는다.

최종 lifecycle 초안은 hardware-owned `SAFE_TRANSITION`/safe generation ack, configure-scoped
session, bounded lease 및 switch/watchdog latch를 사용한다. Bimanual에서는 한 adapter가 두 CAN을
소유하지만 일반 left/right 7축 controller는 독립 운용하며, MoveIt `both_arms`만 단일 14축 custom
FollowJointTrajectory controller가 paired commit/ack를 사용한다. Fake state-machine test는 이 shape를
검증할 뿐 실제 `SystemInterface` integration 완료를 뜻하지 않으므로 3단계 완료 항목은 계속 `[ ]`이다.

- [x] real/mock의 triple-interface claim 흐름을 추적하고 `q_des`, `dq_des`, `tau_ff` 매핑을 문서화한다.
- [ ] 기존 `kp_scale`과 `q_measured` seeding이 activate, deactivate, controller switch, fault recovery에서
      stale target을 방지하는지 테스트한다.
- [x] `kp`, `kd` 기준값 및 scale이 joint별 설정인지, runtime 변경 가능한지, 어느 계층이 소유하는지 확정한다.
- [x] `kp_scale`, `kd_scale`이 현재 URDF-time hardware parameter라 runtime mode switch에 사용할 수 없음을
      기준선으로 삼고 upstream hardware/CAN의 runtime gain 변경 지원 여부를 코드와 실기로 조사한다.
- [x] runtime gain 전략을 (A) mode별 URDF/profile 재기동, (B) Cho adapter에서 runtime MIT command 생성,
      (C) 고정 upstream SHA용 최소 patch/fork 중에서 선택하고 안전성·지연·유지보수 근거를 기록한다.
- [ ] pure torque 진입 시 `kp=0`, `kd=0`의 효과가 backend까지 보장되는지 검증한다.
- [ ] MIT producer(controller)와 consumer(real hardware, MuJoCo, Isaac)의 책임, command lifetime,
      update 주기, ownership 및 atomicity를 아키텍처로 확정한다.
- [ ] triple-interface 유지 또는 최소 확장 결정을 내리고 interface schema(이름, type, 단위, joint order,
      gain/torque 의미, timestamp와 invalid 값 처리)를 문서화하여 controller/backend API를 동결한다.
- [ ] migration 순서를 producer 먼저 구현해 기존 consumer와 호환시키거나, producer와 모든 consumer를
      한 변경 단위로 원자적으로 전환하는 방식 중 하나로 확정한다. 새 consumer가 준비되기 전에 기존
      producer를 제거하지 않는다.

완료 조건:

- 다섯 MIT 값이 각 backend에서 어떻게 생성·전달·소비되는지 한 가지 의미로 설명된다.
- producer/consumer architecture와 versioned interface schema가 승인되며, runtime gain 전략이 결정된다.
- mode 전환 직후 stale `q_des`로 인한 step torque가 발생하지 않는 자동 테스트가 있다.
- 기존 기반과 중복되는 command seeding/interface 구현이 추가되지 않는다.
- 이 설계 게이트가 통과하기 전에는 4~6단계 backend/controller 구현을 시작하지 않는다.

### 4단계 — MuJoCo 및 Isaac Sim MIT effort backend

- [ ] MuJoCo arm actuator를 effort backend로 사용하고 동일한 MIT 식을 control update마다 계산한다.
- [ ] 기존 mode별 MuJoCo scene과 actuator 정의를 목록화하고, effort backend 전환 대상과 기존 scene
      호환/폐기 정책을 정해 position actuator가 조용히 중복 적용되지 않게 한다.
- [ ] Isaac Sim arm actuator에도 같은 계약, 부호, 단위, joint order 및 saturation 순서를 적용한다.
- [ ] Isaac command gate와 `TopicBasedSystem`의 arm/hand command 배열 크기·순서·초기값을 새 schema로
      migration하고 구버전 publisher의 거부 또는 명시적 compatibility adapter를 검증한다.
- [ ] simulator timestep과 controller update rate 차이를 명시하고, 필요하면 zero-order hold/latency를 모델링한다.
- [ ] arm에만 MIT effort semantics를 적용하고 gripper는 기존 position control을 유지한다.
- [ ] mock backend와 실제 sim backend의 역할을 분리해 테스트 목적을 명확히 한다.

완료 조건:

- 동일 MIT 입력과 동일 state에 대해 MuJoCo와 Isaac의 계산 torque가 허용 오차 내에서 일치한다.
- `kp=kd=0`일 때 arm에는 `tau_ff`만 적용된다.
- gripper position command 경로와 기존 동작에 회귀가 없다.
- 기존 MuJoCo scene/actuator 및 Isaac command gate/arm-hand array의 migration 결과가 테스트로 고정된다.

### 5단계 — Controller 운용 경로 완성

- [ ] Position control: `q_des`와 안정적인 `kp`, `kd`를 사용하고 `tau_ff` 기본값을 정의한다.
- [ ] 기존 MoveIt position trajectory 경로가 triple-interface/MIT producer 전환 후에도 같은 joint order,
      interpolation, goal completion 및 cancellation semantics로 동작하는지 호환 검증한다.
- [ ] Velocity control: `dq_des` 중심 제어에서 position 항의 비활성화/hold 정책을 명확히 한다.
- [ ] Impedance control: 목표 pose/joint 상태에서 MIT joint command로 내려가는 변환과 gain 정책을 정의한다.
- [ ] Direct torque control: `kp=kd=0`, `tau_ff=tau_user` 경로를 제공한다.
- [ ] Damped torque control: `kp=0`, `kd>0`, `tau_ff=tau_user`인 기본 torque profile을 제공한다.
- [ ] Compensated torque control: direct/damped base profile을 명시하고 gravity 및 필요 시
      Coriolis/마찰 보상을 `tau_ff`에 합산하는 경로를 제공한다.
- [ ] 각 mode가 동일 controller 내부 profile인지 별도 controller인지 interface claim, 전환 안정성,
      유지보수성을 기준으로 결정한다.

완료 조건:

- 여섯 제어 경로 각각에 launch/config와 최소 재현 예제가 있다.
- direct, damped, compensated torque의 의미와 base profile이 설정 및 topic/API에서 혼동되지 않는다.
- 허용되지 않은 controller 동시 활성화가 controller manager 수준에서 거부된다.
- 기존 MoveIt position trajectory 시나리오가 회귀 테스트를 통과한다.

### 6단계 — 안전 기능 및 실제 모터 전제 검증

- [ ] URDF와 motor 사양을 바탕으로 joint별 position, velocity, torque limit를 적용한다.
- [ ] torque 및 gain command에 slew/rate limit를 적용해 mode 전환 시 불연속을 제한한다.
- [ ] command watchdog을 구현하고 timeout 시 zero/hold/disable 중 mode별 안전 동작을 정한다.
- [ ] command/state의 NaN, Inf, timestamp 이상을 검사하고 안전 상태로 전환한다.
- [ ] controller, Cho adapter, hardware/CAN 중 어느 계층이 최종 torque/gain/velocity limit와 disable을
      보장하는지 지정한다. 상위 계층 실패에도 우회되지 않는 hardware에 가장 가까운 최종 보장 계층을 둔다.
- [ ] mode switch 시 측정 state seeding, gain ramp, controller claim 순서를 원자적으로 처리한다.
- [ ] deactivate 전에 command buffer를 `q_measured`, `dq_des=0`, 안전 gain, `tau_ff=0`으로 갱신하고,
      write loop가 이를 소비했음을 확인한 뒤 interface를 release/disable하는 절차를 정의한다.
- [ ] upstream의 자동 `return_to_zero` 동작을 기본 강제가 아닌 명시적 선택 옵션으로 만든다.
- [ ] 모터가 실제로 MIT mode로 설정되었는지 시작 전에 확인하고, 불일치 시 enable을 거부한다.
- [ ] CAN loss, 일부 joint 응답 누락, E-stop/disable 후 복구 정책을 정의한다.
- [ ] 일부 joint CAN loss 또는 NaN state에서 해당 팔 전체를 disable할지 양팔 전체를 disable할지 정하고,
      bimanual에서 한 팔 fault 발생 시 반대 팔의 hold/controlled-stop/disable 정책을 mode별로 정의한다.
- [ ] 보상 torque에도 joint별 magnitude/rate limit를 적용하고 user torque와 compensation 합산 후 최종
      saturation 순서를 고정한다.
- [ ] simulation/read-only/low-output/full-output 전환 중 자동 판정 가능한 gate와 작업자 수동 확인이
      필요한 gate를 구분하고 수동 확인 기록 없이는 다음 단계 enable이 되지 않게 한다.

완료 조건:

- limits, slew, watchdog, NaN 및 mode-switch fault injection 테스트가 통과한다.
- torque mode 시작 시 중력으로 팔이 낙하할 수 있다는 운용 조건과 지지/정지 절차가 문서화된다.
- MIT mode 미확인 또는 통신 불완전 상태에서는 torque enable이 불가능하다.
- deactivate buffer가 실제 write된 뒤 disable되는 테스트와 partial CAN/NaN fault 주입 테스트가 통과한다.
- user+compensation 합산값은 최종 보장 계층의 magnitude/rate limit를 우회할 수 없다.
- bimanual 한 팔 fault 정책과 자동/수동 gate가 launch/config 및 운용 절차에서 강제된다.

### 7단계 — 단계적 검증과 검증 매트릭스

- [ ] Unit test: MIT 식, mapping, units/sign, saturation, slew, watchdog, seeding을 검증한다.
- [ ] MuJoCo single-arm에서 position → impedance → direct/compensated torque 순으로 검증한다.
- [ ] MuJoCo bimanual에서 namespace, joint mapping, 동시 update와 독립 fault 처리를 검증한다.
- [ ] Isaac Sim에서 같은 single/bimanual 시나리오와 backend parity를 검증한다.
- [ ] Real read-only 단계에서 CAN discovery, motor ID/mode, state sign/order/rate만 확인한다.
- [ ] Real 저출력 단계에서 로봇을 물리적으로 지지하고 한 관절씩 낮은 torque/gain limit로 검증한다.
- [ ] 통과 후에만 다관절 position, compensated torque, impedance 순으로 범위를 확대한다.

아래 매트릭스의 각 cell을 `미지원(N/A, 근거)`, `자동 통과`, `수동 통과`, `실패` 중 하나로 기록한다.
빈 cell은 완료로 간주하지 않는다.

| Backend / 구성 | Position | Velocity | Impedance | Direct torque | Damped torque | Compensated torque |
|---|---:|---:|---:|---:|---:|---:|
| MuJoCo single | [ ] | [ ] | [ ] | [ ] | [ ] | [ ] |
| MuJoCo bimanual | [ ] | [ ] | [ ] | [ ] | [ ] | [ ] |
| Isaac single | [ ] | [ ] | [ ] | [ ] | [ ] | [ ] |
| Isaac bimanual | [ ] | [ ] | [ ] | [ ] | [ ] | [ ] |
| Real single | [ ] | [ ] | [ ] | [ ] | [ ] | [ ] |
| Real bimanual | [ ] | [ ] | [ ] | [ ] | [ ] | [ ] |

완료 조건:

- 각 단계의 로그, 사용 config, commit SHA, pass/fail 결과가 남는다.
- 선행 단계 실패 상태에서는 다음 단계 enable이 차단된다.
- 실제 로봇 저출력 시험에는 즉시 disable 가능한 절차와 관찰자가 포함된다.
- bimanual의 모든 mode에서 한 팔 fault를 주입해 반대 팔 정책과 전체 시스템 종결 상태를 검증한다.
- 자동 gate 결과와 작업자 수동 승인 기록이 모두 필요한 real 단계에서는 두 조건이 함께 충족된다.

### 8단계 — 독립 리뷰 게이트

- [ ] 1~3단계 완료 후 별도 reviewer가 package 경계, submodule 정책, MIT 계약과 중복 구현 여부를 검토한다.
- [ ] 4~5단계 완료 후 별도 reviewer가 simulator parity, controller semantics 및 gripper 회귀를 검토한다.
- [ ] 6단계 완료 후 별도 reviewer가 limits, watchdog, fault recovery와 mode switch 안전성을 검토한다.
- [ ] 실제 저출력 시험 전 별도 reviewer가 launch/config, motor mode check, 비상 정지 절차를 최종 검토한다.
- [ ] 타당한 피드백을 반영하고, 반영하지 않은 의견은 근거와 함께 기록한다.

완료 조건:

- 각 게이트에 reviewer의 지적, 조치, 재검증 결과가 기록된다.
- 안전 관련 미해결 high-severity 지적이 없는 경우에만 실제 구동 단계로 진행한다.

## 4. 구현 원칙

- upstream submodule은 재현 가능한 vendor 기준점으로 취급하고 Cho 프로젝트 고유 정책은 Cho 패키지에 둔다.
- 실제 로봇과 simulator의 공통점은 MIT 명령의 의미와 안전 제한이며, backend 구현 세부는 분리한다.
- controller가 계산한 torque와 backend가 추가하는 PD torque를 명확히 구분해 이중 제어를 방지한다.
- arm과 gripper의 actuator semantics를 분리한다. arm은 MIT/effort, gripper는 position을 유지한다.
- 기존 triple-interface, `kp_scale`, `q_measured` seeding 기반을 먼저 확장하고 검증된 기능을 재작성하지 않는다.
- 실제 로봇 enable은 모든 read-only 및 simulation 검증과 독립 안전 리뷰를 통과한 뒤 수행한다.

## 5. 최종 완료 정의

- `extern/openarm_ros2`의 고정 commit에서 필요한 hardware/CAN 코드만 재현 가능하게 빌드된다.
- Cho description/bringup/controller를 유지한 single 및 bimanual real launch가 제공된다.
- position, velocity, impedance, direct torque, damped torque, compensated torque가 하나의 MIT 계약으로 동작한다.
- MuJoCo와 Isaac arm은 effort backend로 MIT 식을 재현하고 gripper는 position control을 유지한다.
- simulator와 실제 hardware 간 units, sign, joint order, limits 및 mode semantics가 검증된다.
- 모든 안전 시험, 단계적 검증 및 독립 리뷰 게이트를 통과한다.
