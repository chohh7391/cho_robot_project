# VLA — 컨트롤러/서버 보완 및 OpenArm MIT 이식 계획

> 목적: `vla_controller` + `VLAActionServer`를 **"시간 정렬된 참조 스트림의 안전한 추종기"**로
> 재정의하고, 로봇 무관 부분을 `cho_controller_common/vla`로 추출해 Franka와 OpenArm MIT가
> 같은 파이프라인을 쓰게 한다. 기준은 최근 VLA 배포 스택(openpi, GR00T, OpenVLA-OFT, LeRobot)이
> 수렴한 방식이다.
>
> 이 저장소가 쓰는 정책 스택은 **네 개 전부**다: openpi, Isaac-GR00T, OpenVLA-OFT, LeRobot.
> 따라서 `ActionChunk`는 네 스택의 출력 규약을 브릿지 한 층으로 다 받아야 한다(§4).
>
> 2026-09-08 작성. 코드 변경 없음 — 설계만. 착수 순서는 §7.

---

## 0. 이 저장소에서 확인한 사실 (재확인 불필요)

### 현재 구조

- `cho_controller_franka/VLAController` ([vla_controller.cpp](../cho_controller/cho_controller_franka/src/vla_controller.cpp)):
  `control_mode` effort/position/velocity 세 제어 법칙. effort는 task 임피던스 `J^T(Kx·e − Dx·ẋ) + nle`
  또는 joint PD+중력보상, position/velocity는 **개루프 참조 `q_ref_`**를 미분 IK로 적분(측정값 피드백 없음).
- `VLAActionServer` ([vla_action_server.cpp](../cho_controller/cho_controller_franka/src/servers/vla_action_server.cpp), 642줄):
  액션 라이프사이클 + `ActionChunk` 파싱(EMA) + 재생 + 포화(`saturate_des`) + 그리퍼 오케스트레이션 +
  BT 완료 통보 + `/vla/trigger_success` 서비스. **여섯 관심사가 한 클래스**.
- 서버가 `State`에서 실제로 읽고 쓰는 필드는 8개뿐: `H_ee, q_arm`(측정), `H_ee_ref, q_arm_ref`(앵커),
  `H_ee_des, q_arm_des`(출력), `H_ee_init, q_arm_init`(종료 시 래치). → 추출 가능.
- 외부 계약(이 저장소 안): 토픽 `/vla/action/ee_pose`(`ActionChunk`, KeepLast(1) best-effort),
  액션 `/controller_action_server/vla_controller`(`VisionLanguageAction`), 서비스
  `/vla/trigger_success`(성공 GUI·BT waiter가 호출), 클라이언트 `.../vla_controller/notify_completion`(BT).
  `cho_control_tools/vla/action_client.py`는 from_anchor 상대 원궤적 테스터.
- `cho_robot_config/config/openarm.yaml`: `controllers.vla: null`, 레지스트리 role에 `vla` 있음.
- LeRobot 로컬 체크아웃 `~/lerobot` (2026-02-27, `8fff0fde`). `src/lerobot/async_inference/` 5개 파일 1482줄을
  읽고 §2a를 썼다. 블로그(`docs/source/async.mdx`)가 아니라 코드 기준.
- 업스트림 `origin/main` `2774d9bd`(2026-09-07)로 재확인. `async_inference/` 차이는 **import 정리만**(60+/13−), 동작 동일.
  대신 업스트림에는 **RTC가 정책 레벨**(`policies/rtc/`, `policies/common/flow_matching.py`)과 **새 롤아웃 실행기**
  (`rollout/inference/rtc.py`, `policies/rtc/action_queue.py`, `utils/action_interpolator.py`)로 들어와 있다.
  지원 정책: pi0, pi05, pi0_fast, smolvla, groot, evo1, molmoact2. `async_inference/`에는 RTC가 배선되어 있지 않다.
  §2a의 RTC 절은 이 업스트림 코드 기준.

### OpenArm MIT 쪽

- `cho_controller_openarm_mit`는 **프로듀서 전용** 패키지(DESIGN.md). 39개 MIT 커맨드 인터페이스
  (7 조인트 × 5 필드 + 프로토콜 4)를 클레임하고 세션/ACK/lease/SAFE 프로토콜을 `DirectControllerBase`가 소유.
- 출력은 MIT 튜플 `(q_des, dq_des, kp, kd, tau_ff)` 하나. `TaskSpaceImpedanceController`(drive-side, 기본)는
  `q_des = q + J⁺(x_des ⊖ x)`, `dq_des = J⁺ v_des`, kp/kd 고정, `tau_ff = nle + tau_null + tau_limit`.
- **raw-topic 프로듀서 경로는 워치독 초과 시 SAFE 요청** ([direct_controller.cpp:607](../cho_controller/cho_controller_openarm_mit/src/direct_controller.cpp:607)).
  15 Hz 정책 스트림을 그 경로에 물리면 매번 SAFE로 떨어진다. VLA는 `uses_raw_topic() → false` 액션 경로여야 한다.
- `max_reference_offset`이 **임피던스 토크의 유일한 바운드**. 모터가 `kp(q_des − q)`를 컨트롤러가 클램프할 수
  있는 지점 뒤에서 더한다. `torque_limit`은 `tau_ff` 필드만 자른다.
- real MIT 어댑터에는 **핑거 트랜스포트가 없다**(openarm.yaml 주석). 그리퍼 액션은 sim에서만 유효.
- `TaskSpaceImpedanceController`는 `final`이고 Cartesian 헬퍼가 `private`
  ([task_space_impedance_controller.hpp:24](../cho_controller/cho_controller_openarm_mit/include/cho_controller_openarm_mit/task_space_impedance_controller.hpp:24)).
- 제어 주기: mujoco 1000 Hz, real 750 Hz(`controllers_mit.yaml`).

---

## 1. 현재 코드의 결함 (심각도 순)

### ① 청크 입력 무검증 — UB + NaN 영구 오염

`arm_actions`/`gripper_actions`에 `isfinite` 검사가 **없다**(검증되는 건 `inference_frequency`, `control_dt`만).

- **UB**: [vla_action_server.cpp:468](../cho_controller/cho_controller_franka/src/servers/vla_action_server.cpp:468) —
  모르는 `rotation_type` → `dim = 0` → 기대 크기 `chunk_size·0 = 0` → **빈 `arm_actions`가 크기 검사 통과** →
  루프가 빈 벡터에 `begin()+3 ~ begin()+0` 역순 범위로 `orientation` 생성. 메시지 한 개로 컨트롤러가 죽는다.
- **NaN**: 청크에 NaN 하나 → EMA → `SE3::Interpolate`(쿼터니언 전체 NaN) → `H_ee_des` → `saturate_des`
  (`std::min(NaN, lim)`은 NaN 반환) → 미분 IK → `q_ref_`. 출력단 `allFinite()` 가드가 하드웨어는 막지만
  [vla_controller.cpp:355](../cho_controller/cho_controller_franka/src/vla_controller.cpp:355) 주석대로
  `q_ref_`는 복구하지 않는다 → **재활성화 전까지 먹통**.

MIT에서 더 급하다: drive-side는 오염된 `q_des`를 출력단에서 막을 수 없다.

### ② 스트림 staleness 개념 없음

`current_idx >= size()`면 `target_poses.back()` 무한 유지. `goal_timeout_sec_`(60 s)는 **goal** 타임아웃이라
정책이 5초에 죽으면 팔은 **동작 중간 자세**로 55초를 버틴다. 정상 장기 작업과 죽은 정책이 구별 불가.
MIT는 갖고 있고(§0) Franka는 없다.

### ③ 그리퍼가 재생 시각이 아니라 청크 도착 시각에 발사

`apply_gripper_action()`이 **파싱 루프 안**(non-RT 구독 콜백)에서 호출된다
([vla_action_server.cpp:461](../cho_controller/cho_controller_franka/src/servers/vla_action_server.cpp:461)).
"이 청크 안에 닫으라는 웨이포인트가 있으면 지금 닫는다" — EE가 파지 자세에 도달하기 최대 1 추론주기 전에 손이 닫힌다.
튜닝으로 안 잡힌다.

### ④ 재생 클럭이 도착 시각 기준

[vla_action_server.cpp:263](../cho_controller/cho_controller_franka/src/servers/vla_action_server.cpp:263):
`chunk_start_time_ = current_time`, 인덱스 0부터 재생. 정책은 t_obs 기준으로 냈는데 추론 지연 L 뒤에 인덱스 0을
다시 밟으니 로봇은 항상 L만큼 과거 계획을 따르고, 청크 경계마다 lurch. EMA와 포화가 이걸 덮고 있다.

### ⑤ 상대 앵커가 도착 시각 참조

[vla_action_server.cpp:274](../cho_controller/cho_controller_franka/src/servers/vla_action_server.cpp:274):
`rel_anchor_pose_ = state.H_ee_ref`(도착 시각). 정책은 s(t_obs) 기준 델타를 냈으므로 s(t_arr)에 더하면 추론 지연 동안
움직인 만큼 **항상 과이동**. 그리고 상대 모드가 from_anchor 하나 — OpenVLA 계열의 **스텝별 델타**(적분 필요)를 못 받는다.

### ⑥ 속도 피드포워드 없음

파이프라인이 위치만 낸다. velocity 모드는 컨트롤러에서 `(q_ref − q_ref_prev)/dt`로 다시 미분하고, MIT는 `v_des`가
없어 `dq_des = J⁺v_des`를 못 만든다. 시간축 버퍼가 있으면 유한차분으로 공짜다.
→ `CONTROLLER_STABILITY_TODO.md`의 "VLA feedforward for effort/position modes" 미결 항목은 이 문서로 흡수한다.

### 작은 것

| 위치 | 내용 |
|---|---|
| `compute()` `floor(elapsed / dt_)` | `dt_ > 0`이 세 불변식(inference_frequency, chunk_size ≥ 1, control_dt)에 의존, 나눗셈 지점엔 보장 없음 |
| `saturate_des()` | `action_space` 무관하게 pose·joint 둘 다 항상 포화. 비활성 표현의 `prev_des_*`는 goal 시작값에 고정, 표현 전환 시 `initialized_`만 리셋 |
| `compute()` 반환 bool | 컨트롤러가 무시. 死 API |
| `/vla/action/ee_pose`, `/vla/trigger_success` | 하드코딩. joint 청크도 `ee_pose`로 오고, 한 머신에서 두 로봇 VLA 동시 불가 |
| `max_joint_vel_` | position 모드 슬루율 + velocity 모드 속도 상한, 파라미터 하나에 의미 둘 |
| [vla_controller.cpp:415](../cho_controller/cho_controller_franka/src/vla_controller.cpp:415) | `control_mode`를 `RCLCPP_ERROR`로 로깅 |

### 잘 된 것 (유지)

RT/non-RT 경계(`last_shadow_cmd_`로 single-RT-reader 계약 준수), 청크별 상대 앵커 상수화(매 사이클 재앵커 =
runaway, sim 재현), position/velocity 개루프 참조(측정 피드백 루프는 구조적 불안정), 포화의 `√(2·a·dist)` 제동 엔벨로프,
회전 EMA에 측지 보간, `nominal_period()`와 출력단 NaN 가드. 전부 그대로 가져간다.

---

## 2. 업계가 수렴한 구조

```
System 2  VLM/VLA (7~15 Hz)  ──chunk──▶  실행기 / System 1 (50~200 Hz)  ──ref──▶  저수준 추종기 (≥750 Hz)
```

- 실행기는 **시간 인덱스 액션 큐**를 들고 제어 주기로 샘플링한다(LeRobot async inference, openpi 클라이언트).
- 새 청크는 **관측 시각 기준으로 시간축에 이어붙인다**(PI Real-Time Chunking, 2025). 도착 시각 재시작이 아니다.
- 청크 결합은 두 유파: **temporal ensembling**(ACT — 같은 시각 슬롯의 예측을 `wᵢ = exp(−m·i)` 가중 평균)과
  **splice + 짧은 블렌드**(flow-matching 계열 — 멀티모달 액션을 평균내면 두 모드 사이 어디도 아닌 곳으로 가므로 평균을 피함).
- 액션은 대부분 **절대 joint position**(π0 ALOHA, GR00T, LeRobot), 일부 EE 델타(OpenVLA-OFT LIBERO), 일부 joint
  velocity(π0 DROID). 그리퍼는 **연속값**이 표준, OpenVLA 계열만 이진.
- 저수준은 그냥 PD/임피던스. **똑똑할 필요 없고 빠르고 검증 가능해야 한다.** ros2_control 계층이 연구 스택에
  더해줄 가치는 검증·워치독·안전 엔벨로프다 — 연구 스택은 대개 정책이 멈추면 마지막 목표를 그냥 유지한다.

---

## 2a. LeRobot `async_inference` — 코드에서 확인한 것, 가져올 것, 바꿀 것

블로그 설명과 코드가 다른 지점이 있어 코드 기준으로 적는다. 경로는 `~/lerobot/src/lerobot/async_inference/`.

### 실제로 하는 것

1. **정렬은 정수 timestep, wall-clock은 로깅용.** 클라이언트가 관측에 `timestep = 마지막으로 실행한 액션 인덱스`를
   붙여 보내고(`robot_client.py` `control_loop_observation`: `timestep=max(latest_action, 0)`), 서버는 청크 k번째 액션에
   `timestep = i₀ + k`를 매겨 돌려준다(`policy_server.py` `_time_action_chunk`). `timestamp = time.time()`은 지연 로깅에만.
   **클라이언트가 클럭을 소유하고 서버는 인덱스를 에코** — 두 머신 사이 시각 동기가 필요 없다.
2. **도착 시 `timestep ≤ latest_action`인 액션은 버린다**(`_aggregate_action_queues`). 이미 지난 접두사 드롭 = RTC의 그 부분.
3. **큐가 곧 클럭.** 제어 틱마다 정확히 한 개 `get_nowait()`, `sleep`으로 `1/fps` 유지. 보간 없음. 큐가 비면 아무 명령도
   보내지 않는다 — SO-100 서보가 스스로 버틴다.
4. **겹침 합성은 2-청크 고정 가중 블렌드.** `configs.py` `AGGREGATE_FUNCTIONS`: `weighted_average = 0.3·old + 0.7·new`(기본),
   `latest_only`, `average`, `conservative`. ACT식 지수 앙상블이 아니고 슬롯 위치별 감쇠도 없다. 재구성된 큐에는
   **새 청크가 덮는 timestep만 남는다**(옛 큐 꼬리 드롭).
5. **관측 송신 트리거**: `_ready_to_send_observation`: `queue_size / chunk_size ≤ chunk_size_threshold`(g, 0.5~0.6 권장).
   큐가 비면 `must_go`로 서버의 중복 관측 필터(`_obs_sanity_checks`: 같은 timestep 예측됨 / joint-space L2 < 1)를 우회.
6. **안전 계층 없음.** 값 검증·워치독·hold·레이트 리밋 전부 없음. `robot.send_action`에 위임.

### 가져올 것

- **(a) 컨트롤러 클럭 에코 계약 — 가장 중요.** 브릿지 wall-clock을 `header.stamp`에 찍으면 이 저장소의 Multi-PC 구성
  (FastDDS discovery server, PC 두 대)에서 chrony/PTP 없이 틀어진다. LeRobot 방식으로: **브릿지는 관측에 쓴
  `/joint_states`(또는 `~/controller_state`) 메시지의 `header.stamp`를 그대로 `ActionChunk.header.stamp`에 복사**한다.
  컨트롤러 클럭 도메인의 값이므로 정수 timestep 없이 같은 효과. ROS 스탬프가 이미 인덱스 역할을 하니 정수 필드는 두지 않는다.
- **(b) 관측 송신 트리거를 컨트롤러가 뒷받침.** g 규칙은 브릿지 로직이지만 **남은 호라이즌**을 알아야 한다.
  텔레메트리에 `remaining_horizon_sec`, `last_chunk_stamp`(§3).
- **(c) 합성 함수 이름.** 겹침 슬롯의 *값*은 `latest_only | weighted_average(w)`로 LeRobot 이름을 그대로 쓴다. 우리
  cubic 시간 블렌드는 그 값에 *시간축으로* 도달하는 방법이라 직교 — 둘 다 둔다.
- **(d) 새 청크가 덮는 구간 이후는 전부 새 청크**(옛 꼬리 드롭). 우리 splice 규칙과 같다 — 명시만.

### 우리 조건에서 바꿀 것

| LeRobot | 왜 안 맞나 | 우리 |
|---|---|---|
| 큐 = 클럭, 틱당 1 pop (`async_inference`) / 정수 배수 선형 보간 (`rollout`) | 구형 경로는 보간 없음, 신형 `ActionInterpolator(multiplier=N)`는 **인덱스 기반** 선형(액션 사이를 N등분, 시각 아님). 우리는 750~1000 Hz 대 30~50 Hz 격자에 지연·지터가 있어 인덱스 등분으로는 안 맞는다 | 시간 샘플링 + 보간(`ReferenceSampler`), 절대 시각 t_k 기준 |
| 큐 비면 무명령 | 위치 서보는 스스로 버팀. 토크 제어 팔은 안 버팀. MIT는 write 누락 = 프로토콜 폴트 | HOLD 블렌드 + 매 사이클 write |
| 고정 가중 블렌드 | 새 값으로 70% **점프** — 서보가 흡수하는 계단이지 참조 연속성이 아님 | 값 합성 + cubic 시간 블렌드 |
| 검증 없음 | §1 ① | `ChunkValidator` |
| 서버가 "비슷한 관측" 드롭 | 청크가 정상적으로 안 올 수 있음 | `stream_timeout`은 추론 주기의 배수(3×)로 잡아 정상 갭을 폴트로 보지 않음 |

### RTC 실행기 (업스트림 `rollout/inference/rtc.py` + `policies/rtc/action_queue.py`)

우리가 설계하는 실행기와 같은 문제를 푸는 코드다. 사실만 적는다.

- **`ActionQueue.merge(original, processed, real_delay, idx_before)`** — RTC 모드에선 큐를 **통째로 교체**하되 새 청크의
  앞 `real_delay`개를 버린다(`_replace_actions_queue`: "추론 동안 로봇이 이미 실행한 시간"). 비-RTC 모드는 append.
  두 큐를 든다: `original_queue`(정책 공간, 후처리 전)와 `queue`(로봇 공간). **`get_left_over()`는 original을 돌려준다**
  — 즉 RTC 접두사는 정책 자신의 좌표계 값이고, 로봇 측 값이 아니다.
- **`real_delay = ceil(latency_tracker.max() / (1/fps))`** (`_estimate_rtc_delay`). 지연 **예측치**(최근 최대)로 접두사를
  버린다. `_check_and_resolve_delays`는 실제 소비 인덱스 차이(`last_index − idx_before`)와 비교해 **다르면 로그만 찍고
  예측치를 쓴다**. 우리 stamp 정렬은 실제 도착 시각 기준으로 `t_k ≤ now`를 버리므로 이 부분은 우리가 더 정확하다.
- **`queue_threshold`는 절대 스텝 수**(기본 30). 큐 잔량이 이하로 떨어지면 추론 요청. `async_inference`의 비율 g와 다르다.
  우리는 `remaining_horizon_sec`(초)를 내보내고 브릿지가 어느 쪽 규칙을 쓸지 고른다.
- **큐가 비면 `get_action()` → `None` → `note_starved_tick()`, 아무것도 안 보낸다**(`strategies/core.py:366-370`).
  서보가 버틴다는 전제. 우리 HOLD와 대비.
- **`ActionInterpolator(multiplier)`** — 이전 액션과 새 액션 사이를 N등분 선형. 첫 스텝은 보간 없음. 관측 처리·추론 요청은
  `needs_new_action()`이 참인 틱(N틱마다)에만.
- **정책 파라미터**: `predict_action_chunk(obs, inference_delay=d, prev_chunk_left_over=prefix)`. `RTCConfig`:
  `mode: guided|trained`, `execution_horizon: s`(기본 10), `max_guidance_weight`(10.0), `prefix_attention_schedule:
  ZEROS|ONES|LINEAR|EXP`. 가중치는 `[0,d)`=1, `[d,s)` 감쇠, `[s,H)`=0. 제약 **`d ≤ s ≤ H − d`**(`rollout/context.py`).
  `trained` 모드는 `rtc_training_max_delay > 0`으로 학습된 Pi05 체크포인트 전용, 접두사를 hard-inpaint.

### RTC가 우리 계약에 미치는 영향

- **접두사 부기(bookkeeping)는 브릿지 몫이다.** `prev_chunk_left_over`가 정책 공간 값이라 컨트롤러는 만들 수 없고 만들 필요도
  없다. 브릿지가 마지막으로 보낸 청크의 original 배열을 들고 있다가, 컨트롤러가 알려주는 **재생 위치**로 잘라 넘기면 된다.
- 그래서 컨트롤러가 내보내야 할 것은 하나 더: **`playback_stamp`**(샘플러가 지금 재생 중인 절대 시각, §3). 브릿지는
  `t_k > playback_stamp`인 웨이포인트를 `prev_chunk_left_over`로, `d = ceil(측정 지연 / control_dt)`를 `inference_delay`로 넘긴다.
- 컨트롤러 측 splice는 RTC 유무와 무관하게 동일하다. RTC는 새 청크의 **내용**이 접두사와 이어지게 만들고, 우리 stamp 정렬은
  새 청크의 **시각**이 맞게 만든다. 둘은 직교하고 둘 다 필요하다.

---

## 3. 목표 아키텍처

```
cho_interfaces
  ActionChunk v2 ─ header.stamp = t_obs, seq, relative_mode, joint_names[], gripper_mode, arm_velocities[] (§4)

cho_controller_common/vla            ◀ 로봇 무관. rclcpp 없이 컴파일(시간은 double, 청크는 내부 POD)
  ChunkValidator     크기·finite·rotation_type·control_dt>0·joint 윈도우·워크스페이스 박스 → 드롭 + 카운터
  ActionBuffer       시간 인덱스 슬롯. t_k = t_obs + k·dt. 도착 시 t_k ≤ now 접두사 버림, 나머지 splice.
                     옵션 ensembling. 관측시각 앵커용 참조 히스토리 링(ref(t) 조회).
  ReferenceSampler   sample(now) → q_des, dq_des, x_des, v_des, gripper_des
                     joint 선형 / SE3 측지 보간, 속도는 유한차분(arm_velocities 있으면 그것)
  ReferenceLimiter   현재 saturate_des(속도+가속 사다리꼴) + joint 윈도우 클램프. action_space별로만 실행
  StreamWatchdog     RUNNING ─(now − t_last_chunk > stream_timeout)→ HOLD ─(hold_timeout)→ ABORT
                     HOLD 진입 = 현재 참조로 cubic release(마지막 웨이포인트 동결 아님). 재도착 시 RESUME(옵션)
  Telemetry          청크 지연(t_arr − t_obs), 나이, 드롭 수·이유, splice 불연속 크기, 큐 깊이,
                     remaining_horizon_sec, last_chunk_stamp (브릿지의 g 트리거용, §2a-b),
                     playback_stamp (브릿지의 RTC 접두사 부기용, §2a RTC)

cho_controller_franka/VLAController       세 제어 법칙 그대로. 입력만 sampler에서. dq_des를 velocity ff로
cho_controller_openarm_mit/VlaController   drive-side 법칙 그대로. x_des/v_des(또는 q_des/dq_des)만 sampler에서

VLAActionServer (non-RT, 얇게)  goal/cancel, ActionChunk→POD 변환+submit, success 트리거, BT 통보, 텔레메트리 publish
                                 토픽/서비스 이름은 파라미터(기본값 = 현재 이름, 하위호환)
```

`cho_controller_common/vla`가 **rclcpp 없이** 컴파일되는 것이 핵심이다. 그래야 CM 픽스처 없이 gtest로
splice·워치독·NaN 드롭·per_step 적분을 수만 케이스 돌릴 수 있다. `cho_controller_common`에 `cho_interfaces` 의존은
넣지 않는다(변환은 서버 쪽).

7-DoF 고정은 **1차에서는 유지**한다(Franka·OpenArm 단일 팔 모두 7). `joint_names[]`는 순서 재배열과 검증에만 쓰고,
바이매뉴얼(14+2)은 동적 크기로 가는 2차 작업. GR00T의 named-group 출력이 자연스럽게 여기로 온다.

---

## 4. `ActionChunk` v2와 정책별 매핑

### 필드

```
std_msgs/Header header      # stamp = t_obs. 브릿지 wall-clock이 아니라 관측에 쓴 joint state 메시지의 stamp를
                            # 그대로 에코(컨트롤러 클럭 도메인, §2a-a). 0이면 도착 시각 폴백 = 현재 동작
uint64 seq                  # 브릿지 단조 증가. 0이면 서버가 도착 순서로 부여
string action_space         # "joint" | "task"
string relative_mode        # "absolute" | "from_anchor" | "per_step". 비어 있으면 relative 플래그로 폴백
bool relative               # 하위호환. relative_mode가 비어 있을 때만 참조
string rotation_type        # euler | axis_angle | rotation6d | quaternion (task 전용)
string[] joint_names        # 비어 있으면 robot_config 기본 순서. 채워지면 재배열+검증(누락·중복 거부)
int32 chunk_size
float64 control_dt          # 웨이포인트 간격. ≤0이면 (1/inference_frequency)/chunk_size 폴백 + WARN
float64[] arm_actions       # [chunk_size × dim] row-major
float64[] arm_velocities    # 선택. 같은 배열형. 비어 있으면 유한차분
string gripper_mode         # "binary" | "continuous". 비어 있으면 binary = 현재 동작
float32[] gripper_actions   # binary: 부호(<0 close, >0 open). continuous: 개방률 [0,1]
```

기존 필드는 그대로, 새 필드는 빈 값이 현재 동작이다. **현재 정책 브릿지는 깨지지 않는다.** 단, ④⑤의 이득은
브릿지가 `header.stamp`를 t_obs로 채울 때만 나온다(§5).

### 정책 스택별 매핑

문헌·공개 코드 기준. 브릿지 작성 시 **사용 중인 버전에서 재확인** — 체크포인트마다 액션 공간이 다르다.

| 스택 | 전송 | 액션 규약 (대표 임베디먼트) | `action_space` | `relative_mode` | `gripper_mode` | 호라이즌/주기 |
|---|---|---|---|---|---|---|
| **openpi** (π0/π0.5) | websocket + msgpack, `actions [H, D]` | ALOHA: 절대 joint pos 14(그리퍼 포함). DROID: joint **velocity** 7 + 그리퍼 pos 1 | joint | ALOHA `absolute`; DROID `per_step`(브릿지가 v·dt로 변환) | continuous | H=50 @ 50 Hz(ALOHA); DROID 15 Hz, `open_loop_horizon`으로 일부만 실행 후 재추론 |
| **Isaac-GR00T** (N1/N1.5) | ZMQ, `action.<group>` dict | 임베디먼트별 modality config. 보통 절대 joint pos, 그룹별 배열(left_arm/right_arm/hands) | joint | `absolute` | continuous | H=16 @ 데이터셋 fps(~20 Hz) |
| **OpenVLA-OFT** | HTTP JSON (`json_numpy`) | LIBERO: EE 델타 6(xyz + euler rpy) + 그리퍼 1, **스텝별 델타**. ALOHA: 절대 joint 14 | task / joint | LIBERO `per_step`; ALOHA `absolute` | LIBERO **binary**; ALOHA continuous | LIBERO K=8, ALOHA K=25, 청크 개루프 실행 후 재추론 |
| **LeRobot** (ACT/DP/SmolVLA/π0) | gRPC async(`PolicyServer`/`RobotClient`) 또는 동기 | 절대 joint pos + 그리퍼 조인트 | joint | `absolute` | continuous | ACT 100(ensembling m=0.01), DP 16/실행 8, SmolVLA 50; async는 timestep 정렬·`aggregate_fn` |

읽는 법:

- `joint_names[]`는 **GR00T의 그룹 출력**과 **LeRobot의 모터 순서**를 이 저장소의 `robot_config` 순서로 정렬해 주는 필드다.
  브릿지가 매핑을 안 하고 이름만 붙여 보내면 서버가 재배열한다.
- `per_step`은 **OpenVLA-OFT LIBERO(EE)**와 **π0 DROID(joint velocity)** 둘을 위한 것이다. 서버가 관측시각 앵커
  `ref(t_obs)`에서 누적 적분한다. joint velocity는 브릿지가 `Δq_k = v_k·control_dt`로 바꿔 보낸다 — 서버에 velocity
  action_space를 하나 더 두지 않는다(같은 정보, 표현만 다름).
- **ensembling 기본 OFF.** ACT에만 켠다. π0/SmolVLA/GR00T(flow-matching·diffusion)는 splice + 블렌드.
- `binary` 그리퍼는 OpenVLA-OFT LIBERO만 남는다. 나머지는 연속 개방률을 `cho_controller_gripper` 폭[m]으로 매핑.
- 네 스택 **모두 전송 계층에 wall-clock 타임스탬프가 없다**(LeRobot async는 정수 timestep 에코). t_obs는 **브릿지가**
  관측에 쓴 joint state의 stamp를 에코해 넣는다(§5). 브릿지 자기 시각이 아니다.

---

## 5. 브릿지 계약 (이 저장소 밖 코드가 지켜야 할 것)

브릿지 = 정책 서버(websocket/ZMQ/HTTP/gRPC)와 ROS 사이의 노드. 스택마다 하나, 출력은 전부 `ActionChunk` v2.

1. 관측에 쓴 joint state 메시지의 `header.stamp`를 `ActionChunk.header.stamp`에 **그대로 복사**한다. 브릿지의
   `now()`도, 추론이 끝난 시각도 아니다. 컨트롤러 클럭을 에코하는 것이라 PC 간 시각 동기가 필요 없다(§2a-a).
2. `control_dt`를 명시한다. 정책 설정의 fps/주기에서 가져온다.
3. `seq`를 단조 증가시킨다. 재추론으로 이전 청크를 덮을 때도 증가.
4. 조인트 순서가 로봇 기본 순서와 다르면 `joint_names[]`를 채운다.
5. 상대/절대·그리퍼 모드를 **명시**한다(`relative_mode`, `gripper_mode`). 폴백에 의존하지 않는다.
6. 정책 서버가 죽거나 멈추면 발행을 멈춘다. 마지막 청크를 반복 발행하지 않는다 — 워치독이 그걸 감지해야 한다.
8. 관측 송신 시점은 텔레메트리의 `remaining_horizon_sec / chunk_duration ≤ g`로 정한다(LeRobot `chunk_size_threshold`,
   0.5~0.6 권장). 매 틱 보내는 것(g→1)도, 다 소진되고 보내는 것(g→0, 동기 추론)도 아니다.
7. (RTC 지원 정책 — pi0/pi05/smolvla/groot 등) 접두사 부기는 브릿지가 한다: 마지막 청크의 **정책 공간(original) 배열**을
   보관하고, 텔레메트리 `playback_stamp` 이후의 웨이포인트를 `prev_chunk_left_over`로, `ceil(측정 지연 / control_dt)`를
   `inference_delay`로 `predict_action_chunk()`에 넘긴다. 컨트롤러는 시간 정렬과 재생 위치 공개만 한다(§2a RTC).

`cho_control_tools/vla/`에 스택별 브릿지 골격을 두는 것은 이 문서 범위 밖(별도 TODO).

---

## 6. OpenArm MIT — `VlaController`

### 구조

```cpp
class VlaController final : public TaskSpaceImpedanceController
```

- `TaskSpaceImpedanceController`의 `final` 해제, Cartesian 헬퍼 `private → protected`,
  `write_task_target(control_time, dt, target)`를 **virtual**로. VLA는 이것만 override:
  `sample_pose_trajectory(start, goal, u)` 대신 `ReferenceSampler.sample(now)`.
- 상속으로 따라오는 것: 39 인터페이스 클레임, 세션/ACK/lease/SAFE, `home 1` 시동 램프, drive-side 임피던스,
  null-space posture, joint-limit 스프링, 마찰 FF, `gravity_scale`, `max_reference_offset`, release 블렌드.
- `action_space: joint`는 `q_des = 샘플된 관절값`(프로파일 윈도우 + `max_reference_offset` 클램프),
  `dq_des = 샘플된 속도`(`command_velocity` 클램프), `tau_ff = nle`. task보다 단순.
- `direct_controller.hpp:73`의 정신 그대로: *"differs only in where q_des/dq_des originate."*

### MIT에서만 터지는 함정

1. **raw-topic 경로 금지.** 워치독 → SAFE. 액션 경로로, 내부 참조에서 **매 사이클** 튜플을 쓴다. 청크 부재 = hold, write 건너뜀 = 프로토콜 폴트.
2. **`max_reference_offset` 필수.** VLA 설정에서 비워두면 configure 실패로 만든다(기본 파생값에 기대지 않는다).
3. **NaN 가드는 입력단**(ChunkValidator). drive-side는 출력단에서 못 막는다.
4. **정책 타임아웃에 SAFE 요청 안 함.** 태스크 컨트롤러 결정과 동일 — HOLD는 `release_duration` 블렌드, 서버 유지. FK/dynamics 실패만 SAFE.
5. **real은 그리퍼 없음.** `gripper_actions`는 real에서 WARN 1회 + 무시. 없는 액션 서버로 디스패치 금지.
6. 재생 클럭은 `nominal_period` 규칙(CLAUDE.md). 750 Hz real / 1 kHz mujoco.

### 배선

| 대상 | 할 일 |
|---|---|
| `controller_plugins.xml`, `CMakeLists.txt` | `VlaController` 등록·소스 추가 |
| `config/{mujoco,real}/controllers_mit.yaml` | `vla_mit_controller` 블록(태스크 컨트롤러 파라미터 + VLA 파라미터) |
| `utils/launch_utils.py` | `MIT_DIRECT_CONTROLLERS`, `RETURN_TO_ZERO_MIT_CONTROLLERS`에 추가. `REAL_MIT_DIRECT_CONTROLLERS`는 **sim 검증 후** |
| `bringup_mujoco_robot.launch.py` | `mit_controller_name` choices |
| `cho_robot_config/config/openarm.yaml` | `controllers.vla: vla_mit_controller`, `actions.preferences` |
| `test/test_vla_controller.cpp` | 기존 CM 픽스처 패턴. 세션/ACK/SAFE 불변, 청크 부재 hold, NaN 드롭, 워치독 HOLD |

---

## 7. 단계

**전 단계 2026-09-09 완료.** 결과는 §7a(1단계)와 §7b(2~5단계).

| 단계 | 내용 | 해소 | 검증 |
|---|---|---|---|
| 1 | ~~`cho_controller_common/vla`~~ → **`cho_controller/utils/cho_vla_core` 별도 패키지. 완료** (§7a) | ①②③⑥ | gtest 87개 |
| 2 | **완료.** Franka `VLAActionServer`를 코어 어댑터로 축소 | 구조 | 빌드 통과. sim 실행은 사용자 몫 |
| 3 | **완료.** `ActionChunk` v2 + `VisionLanguageAction` v2 + `VlaTelemetry` 신설 | ④⑤ | gtest. 브릿지는 저장소 밖 |
| 4 | **완료.** `VlaController` (task/joint 모두 impedance) | MIT | CM 픽스처 gtest 12개 → mujoco 실행은 미검증 |
| 5 | **완료.** 연속 그리퍼, 합성 가중치, 텔레메트리 토픽, 이름 파라미터화 | 다듬기 | 435개 테스트 통과 |

1→2가 끝나면 Franka는 결함 ①②③⑥이 사라진 상태로 **동작은 동일**해야 한다. 3에서 처음으로 동작이 바뀐다
(지연 보상). 2와 3을 합치지 않는 이유가 그것이다 — 회귀와 개선을 한 diff에 섞지 않는다.

---

## 7a. 1단계 결과 (2026-09-08)

`cho_controller/utils/cho_vla_core` — 구현 1615줄, 테스트 1216줄, gtest **79개 통과**.
설계 근거는 패키지의 `DESIGN.md`.

**계획에서 바뀐 것: 위치.** `cho_controller_common` 안의 모듈이 아니라 별도 패키지로 만들었다. 측정된 이유 둘.

1. `cho_controller_common`은 `-Ofast`로 컴파일된다. `-Ofast`는 `-ffinite-math-only`를 함의하고, 이건
   `std::isfinite()`를 `true`로 접는다. g++ 11.4에서 실측: `-Ofast` 빌드는 NaN을 담은 벡터를 all-finite로
   보고한다(경고 없음). 즉 결함 ①을 고치려는 검증기가 그 자리에서 정확히 그 결함을 갖게 된다.
   저장소 전체에서 `-Ofast`를 쓰는 패키지는 이것 하나뿐이고, `allFinite()` 가드를 가진 패키지들
   (franka / openarm_mit / hardware)은 전부 기본 최적화라 **기존 가드는 안전하다.**
2. `cho_controller_openarm_mit`은 `cho_controller_common`을 의존하지 않는다. 파이프라인을 쓰려고 그
   의존을 추가하면 eiquadprog와 TSID 유래 솔버 스택이 최소 프로듀서 패키지로 끌려온다.

새 패키지는 `-fno-finite-math-only`를 명시하고, 이 플래그는 **순서와 무관하게 `-Ofast`를 이긴다**(실측).
`test_finite_math_guard`가 그 플래그 자체를 지킨다 — 플래그를 지우고 `-Ofast`를 넣으면 실패하고,
플래그가 있으면 `-Ofast`를 뒤에 붙여도 통과하는 것까지 확인했다.

**구성**

| 파일 | 역할 |
|---|---|
| `types.hpp/cpp` | POD + fail-closed 문자열 파싱 + 회전 디코딩. `ActionChunk` **v2 모양을 미리 반영**해서 step 3이 필드 채우기로 끝나게 함 |
| `chunk_validator.hpp/cpp` | `validate()`(구조·finite·permutation·seq) → `decode()`(앵커 적용·윈도우·워크스페이스) → `ingest()` |
| `action_buffer.hpp/cpp` | 시간 인덱스 타임라인, splice, 값 합성, cubic 시간 블렌드, `sample_timeline()` |
| `reference_history.hpp/cpp` | 관측시각 앵커용 seqlock 링 (RT 쓰기 / executor 읽기) |
| `reference_limiter.hpp/cpp` | 속도·가속 사다리꼴 + joint 윈도우 클램프 |
| `stream_watchdog.hpp/cpp` | `kWaitingFirstChunk → kRunning → kHold → kAborted` |
| `gripper_dispatch.hpp/cpp` | 샘플된 값에 대한 엣지 트리거 + `retry()` |

**설계 판단 넷 (계획에 없던 것)**

- **`Timeline`을 값으로 분리.** `splice()`는 할당하므로 executor, `sample_timeline()`은 읽기만 하므로
  호스트가 `Timeline` 스냅샷을 자기 `RealtimeBuffer`로 넘겨 RT에서 할당 없이 샘플링. 같은 코드가 양쪽을
  서비스하므로 단일 스레드 테스트가 실제 RT 경로를 검증한다.
- **`ReferenceHistory`가 유일한 동시 구조.** 슬롯마다 짝/홀 시퀀스 카운터. 실제 writer 스레드 대상
  20만 회 읽기에서 torn 0 확인(`test_reference_history`).
- **과거 접두사의 마지막 웨이포인트를 세그먼트 원점으로 보존.** 없으면 splice 직후 첫 샘플이 첫 미래
  웨이포인트로 점프한다. 청크가 전부 과거여도 hold 타깃 하나는 남는다.
- **호라이즌 소진 후 속도는 0.** 마지막 세그먼트 속도를 유지하면 MIT 드라이브의 `kd*(dq_des − dq)`가
  정지 타깃을 계속 밀어붙인다.

**구현 중 실측으로 잡은 것 둘**

1. `rotation_dim()`은 잘못된 enum에 0을 주지만 `task_waypoint_dim()`이 `3 + 0 = 3`을 반환해서
   내가 처음 넣은 `dim == 0` 가드가 죽은 코드였다. 회전 **블록** 크기를 검사하도록 고쳤다.
2. 테스트가 틀렸던 케이스: `AngleAxisd` 추출 각도는 [0, π]로 접히는데 무한히 커지는 값을 그대로
   비교해 π 이후 전부 torn으로 오판했다. seqlock은 정상이었다.

**아직 안 한 것 (step 2 이후)**

- 어느 호스트에도 배선하지 않았다. Franka `VLAActionServer`는 그대로다.
- `cho_interfaces`는 손대지 않았다. `ActionChunk` v2와 `VisionLanguageAction` 개편은 step 3(§8 열린 것).
- 텔레메트리는 `Telemetry` 구조체만 있고 publish 경로는 없다.

---

---

## 7b. 2~5단계 결과 (2026-09-09)

테스트 **435개 통과, 실패 0** (`cho_vla_core` 87, `cho_controller_openarm_mit` 100,
`cho_task_manager` 110, `cho_bringup_openarm` 60, `cho_robot_config` 78).

### 2단계 — Franka 어댑터

`VLAActionServer` 642줄 → 코어 어댑터. 남은 것은 ROS 모양의 일뿐이다: 액션 라이프사이클,
`ActionChunk`↔POD 변환, 그리퍼 액션 클라이언트, BT 완료 서비스, 텔레메트리 발행.
스레드 분담을 코드에 명시했다 — executor(validator·buffer·smoother, `splice()`는 할당),
RT(`sample_timeline`·limiter·watchdog·gripper edge, 할당 없음), 공유(`ReferenceHistory` seqlink + 방향이
문서화된 atomics).

**동작 동일성을 위해 판단한 것 둘.**

- `chunk_time_source` 파라미터 신설, 기본 `arrival`. 코어는 본질적으로 관측시각 정렬이지만
  브릿지가 컨트롤러 클럭을 에코하도록 고쳐지기 전에 stamp를 믿으면 모든 웨이포인트가 엉뚱한
  epoch에 놓인다. 이건 임시 발판이 아니라 **영구 계약**이다 — 동기화 안 된 브릿지는 `observation`을
  쓰면 안 된다.
- `chunk_ema_factor`를 제거하지 않고 코어에 `apply_ema()`로 옮겼다(테스트 6개). 노이즈 있는 정책을
  위한 정당한 스무딩이고, 말없이 없앨 것이 아니다. 다만 이게 가리던 것 대부분은 도착시각 재시작
  lurch였으므로 `observation`에서는 1.0(비활성)로 둘 수 있다.

**RT에서 할 수 없는 일 하나를 옮겼다.** 그리퍼 엣지 판정은 샘플된 값에 대해 RT에서 해야 하지만
`async_send_goal()`은 제어 루프에서 부를 수 없다. RT가 atomic에 명령을 적고 5 ms non-RT 타이머가
보낸다.

### 3단계 — 메시지

`ActionChunk` v2: `seq`, `relative_mode`, `joint_names[]`, `arm_velocities[]`, `gripper_mode` 추가.
기존 필드 유지, 새 필드는 빈 값이 현재 동작이라 **브릿지가 깨지지 않는다.** `header.stamp` 의미를
"관측에 쓴 joint state의 stamp를 그대로 에코"로 문서화했다.

`VisionLanguageAction` v2: `task`(언어 지시문 — 없어서 BT가 지시를 말할 수 없었다), `stream_timeout`,
결과에 `string message`(TaskSpace/JointSpace엔 있는데 VLA만 없었다), 피드백을 한 번도 발행되지 않던
`percent_complete` → `remaining_horizon_sec` + 청크 카운터로 교체.

`VlaTelemetry` 신설. 브릿지가 실제로 필요한 건 둘이다: `remaining_horizon_sec`(다음 관측 시점),
`playback_stamp`(RTC 접두사 절단 지점).

### 4단계 — `VlaController`

`TaskSpaceImpedanceController`를 상속하고 **`write_task_target()` 하나만** override 한다.
베이스에 가한 변경은 최소다: `final` 해제, 멤버 `private`→`protected`, `write_task_target` virtual화,
`uses_task_space_action()` 신설(두 서버가 같은 39 인터페이스를 몰지 못하게).

**두 action space 모두 impedance** — 이 드라이브가 할 수 있는 유일한 것이다.

| | q_des | dq_des | tau_ff |
|---|---|---|---|
| task | `q + J⁺(x_des ⊖ x)` | `J⁺ v_des` | `nle + tau_null + tau_limit` |
| joint | 샘플된 관절값 (offset·윈도우 클램프) | 샘플된 속도 (`command_velocity` 클램프) | `nle` |

joint 경로가 오히려 단순하다: Jacobian 없음, 유사역행렬 없음, 특이점 없음.

**설정 단계에서 거부하도록 만든 것 둘** (베이스는 둘 다 허용한다).

- `max_reference_offset` 필수. 베이스는 `0.5·torque_limit/kp`로 파생하는데, 그건 운영자가 쓴
  TaskSpace goal에는 합당하고 신뢰할 수 없는 정책 출력에는 아니다. 드라이브가 `kp(q_des−q)`를
  이 컨트롤러가 클램프할 수 있는 지점 **뒤에서** 더하므로 이것이 임피던스 토크의 유일한 바운드다.
  설정값은 `min(0.25·torque_limit/kp, 0.15 rad)` — 토크 바운드만으로는 joint 2에 1.5 rad을 허용하는데
  그건 임피던스 오프셋이 아니라 lunge다.
- `stream_timeout_sec > 0` 필수. 없으면 정책이 죽었을 때 팔이 동작 중간 Cartesian 참조를 드라이브의
  전체 강성으로 무한히 붙잡는다.

**SAFE를 요청하지 않는다.** 스트림 타임아웃·취소·성공 모두 `release_duration` cubic 블렌드로 참조를
측정 자세로 풀고 서버는 살려둔다. FK/dynamics/capacity 실패만 SAFE — 베이스와 동일한 판단이다.

### 5단계

연속 그리퍼(데드밴드 포함), `chunk_aggregate_weight`(LeRobot `weighted_average` 등가), 텔레메트리
토픽, 이름 파라미터화(`chunk_topic`/`success_service`, 기본값은 역사적 전역 이름 유지).
`vla_completion_service_name(controller)`가 로봇 인식형이 됐다 — 컨트롤러가 자기 액션 이름에서
서비스를 파생하므로 Franka(`vla_controller`)와 OpenArm(`vla_mit_controller`)이 다르다.

### 구현 중 테스트가 잡은 것 셋

1. **`max_task_wrench`는 drive-side에서 무효라도 항상 필수 검증 대상이다.** 두 config 블록이 이걸
   빼고 있어서 실제로 `on_configure`에 실패할 상태였다. CM 픽스처가 잡았고, 이후 같은 종류를 잡도록
   config 블록이 컨트롤러의 설정 요구를 만족하는지 검사하는 pytest를 추가했다.
2. `task_start_time_`을 VLA goal 시작 시 설정하지 않아서 `goal_timeout_sec > 0`이면 마지막 TaskSpace
   goal 시점 기준으로 측정됐다.
3. `build_chunk` 실패 시 reject 사유를 항상 `kRotationType`으로 기록했다 → `kUnparseableField` 신설.

---

## 7c. MuJoCo 검증 (2026-09-09)

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque \
  mit_controller_name:=vla_mit_controller
ros2 run cho_control_tools vla_mit_probe
```

`cho_control_tools/vla/mit_probe.py` 신설(`vla_mit_probe`). 추론 브릿지처럼 청크를 흘려보내고
팔과 텔레메트리가 **실제로** 무엇을 했는지 단정한다 — CM 픽스처가 덮지 못하는 절반이다.
**18/18 통과.**

| 확인 | 결과 |
|---|---|
| joint space 추종 | q1 −0.0008 → **정확히 +0.3500** (목표 0.35), 60개 수락 0개 거부 |
| task space 추종 | TCP x +0.0019 → **+0.0405** (목표 +0.0419), 횡방향 \|dy\|=1.6 mm \|dz\|=3.6 mm |
| action space 전환 | joint → task, 텔레메트리도 따라 바뀜 |
| NaN 청크 | 거부, `last_reject=non_finite`, 수락 카운터 불변 |
| 미지 `rotation_type` + 빈 배열 (구 UB 경로) | 거부, `last_reject=unparseable_field` |
| 갭 후 복구 | `hold` → `running` |
| 죽은 스트림 | `hold` → abort, 메시지가 이유를 설명 |
| abort 후 재사용 | 새 goal 수락 후 다시 구동 |

### 시뮬레이터가 잡은 결함 둘 — 단위 테스트로는 잡을 수 없었다

**① `resume_on_chunk` 기본값이 틀렸다.** 첫 실행에서 4개가 실패했고 원인은 하나였다. malformed
청크만 발행되는 구간은 워치독을 갱신하지 않으므로 200 ms 뒤 `hold`로 갔는데, `resume_on_chunk`가
기본 false라서 이후 **정상 청크가 와도 `running`으로 복귀하지 못했다.** 즉 `hold`가
`hold_timeout` → abort까지 사실상 종결 상태였고, 15 Hz BEST_EFFORT 스트림에서 두 개 연속 유실이면
rollout이 끝난다.

내가 `false`로 둔 근거("정책이 왜 멈췄는지 컨트롤러는 모른다")가 틀렸다. 정책이 **실제로** 죽은
경우는 `hold_timeout`이 이미 처리한다 — 죽었으면 더 이상 청크가 오지 않는다. latch가 추가로 하는
일은 **일시적 갭으로 goal을 죽이는 것**뿐이다. 그리고 복귀는 계단이 아니다: hold 진입 시 참조가
측정 자세로 released 되고 돌아온 청크는 `blend_duration`으로 splice 된다.

→ 호스트 파라미터 `resume_on_stream_recovery` 신설, **두 호스트 모두 기본 true**. 라이브러리
기본값은 false로 남겨 호스트가 명시적으로 선택하게 했다. 코어에 회귀 테스트 2개 추가(전이 갭
반복 회복 / 복귀를 켜도 hold_timeout은 여전히 abort 한다).

**단위 테스트가 이걸 놓친 이유가 중요하다.** `StreamWatchdog.ResumeIsOptIn`이 두 분기를 다
검증하고 있었다 — 즉 latch 동작을 *설계대로* 맞다고 단정했다. 테스트는 내 설계를 검증했고,
설계가 틀렸다.

**② 과거 접두사 판정이 `t <= now`였다.** 도착시각 경로에서 `t_obs = arrival`이라 웨이포인트 0이
정확히 `now`에 놓이고 **매 청크에서 하나씩 버려졌다.** 측정: 수락 15개에 `dropped_past`가 정확히 15.
그러면 이 카운터는 추론 지연 경보로서 쓸모가 없다(항상 청크당 1). 지금 실행될 웨이포인트는 "과거"가
아니므로 `t < now`가 맞다. 고친 뒤 `dropped_past=0`. 테스트 1개 추가.

### 여전히 안 한 것
- **real bringup에서 선택 불가.** `REAL_MIT_DIRECT_CONTROLLERS`에 넣지 않았다. real config에 블록은
  있지만 검토용이다. MuJoCo 검증은 끝났으니 이제 그 결정을 할 수 있는 상태다.
- **바이매뉴얼 미지원.** 7-DoF 고정이다.
- **실제 정책으로 돌려보지 않았다.** 프로브는 합성 청크를 쓴다. 브릿지를 붙여 실제 openpi/GR00T
  출력으로 돌리는 것은 별개의 검증이다.
- **OpenArm VLA 행동 트리가 없다.** BT VLA 흐름은 여전히 Franka 전용이다.
- **브릿지 넷은 저장소 밖이라 손대지 않았다.** ④⑤의 이득은 브릿지가 `header.stamp`를 에코하고
  `chunk_time_source: observation`으로 바꿀 때 나온다.

## 8. 결정한 것 / 안 하는 것

- **추출한다** (MIT 로컬 재작성 아님). 결함 ①②③이 추출 대상 코드 안에 있어 같은 작업이다. 2026-09-08 결정.
- **task + joint 둘 다** 지원. joint가 오히려 단순하고 네 스택 중 셋이 joint다.
- **ensembling 기본 OFF.** 켜는 건 ACT 체크포인트 한정.
- **7-DoF 고정 유지**(1차). 바이매뉴얼 동적 크기는 2차.
- **Franka 세 제어 법칙은 손대지 않는다.** MIT에 이식하지도 않는다 — MIT는 튜플 하나다.
- **코어는 별도 패키지**(`cho_vla_core`), `cho_controller_common` 안이 아니다. 2026-09-08 결정, 근거는 §7a.
- **`VisionLanguageAction`도 개편 대상**(step 3): `percent_complete`는 한 번도 발행되지 않는 죽은 필드이고
  VLA 롤아웃엔 끝점이 없어 정의 불가 → `remaining_horizon_sec`로 교체. `string message`가 빠져 있어
  (TaskSpace/JointSpace엔 있음) BT가 실패 이유를 구별 못 함 → 추가. **언어 지시문 필드가 없음** → `string task`
  추가. `inference_frequency`는 `control_dt` 폴백으로만 남긴다.
- **외부 이름(`/vla/trigger_success`, `/vla/action/ee_pose`, `.../notify_completion`)은 기본값 유지**, 파라미터화만.
  BT waiter와 success GUI가 의존한다.
- `hold_on_stale`의 RESUME(청크 재도착 시 HOLD → RUNNING 복귀)은 **옵션, 기본 OFF**. 정책이 왜 멈췄는지
  컨트롤러는 모른다. 기본은 HOLD → hold_timeout → ABORT.

### 열린 것

- `control_dt ≤ 0` 폴백을 언제 제거할지(WARN → 거부). 브릿지 넷이 모두 명시하게 된 뒤.
- 워크스페이스 박스의 기준 프레임과 값. 로봇별 `robot_config`에 두는 게 맞아 보이나 미정.
- 텔레메트리 메시지 타입(`cho_interfaces/msg/VlaTelemetry` 신설 vs `diagnostic_msgs`).
