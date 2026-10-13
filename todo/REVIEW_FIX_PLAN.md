# 전체 리뷰 수정 계획 (Pour 제외)

- 출처: review-team 전체 리뷰 `20261007-212552_main` (HEAD `720dba3`), 리포트 `.review-team/reviews/20261007-212552_main/report.md`
- 항목 ID(M·S·C·ST·Q)는 리포트의 ID를 그대로 쓴다.
- **이 계획에서 뺀 것**
  - Pour 관련 항목 전부: M2, S1, C4(pour 액션 이름 사본), 감사 관찰 두 건(`pour_client.py` Ctrl-C, `pouring_controller` 의 `on_deactivate`).
    - pour 코드는 main 에서 지웠다. 리뷰 시점 그대로 `auto_chem` 브랜치에 남아 있다.
    - 그 브랜치에서 pour 를 다시 살린다면 위 항목을 그쪽에서 다룬다.
  - S2: 감사에서 FAIL 이 나 리포트에서도 빠졌다.
- 줄 번호는 리뷰 시점(HEAD `720dba3`) 기준이다. 고치기 전에 다시 확인한다.
- 진행: 1–3단계와 ST-3 완료(이 파일을 추가한 커밋). 4단계부터 남았다.

## 0. 순서

| 단계 | 항목 | 왜 이 순서인가 |
|---|---|---|
| 1 | M1 CI 복구 | 지금 CI 가 테스트를 하나도 돌리지 않는다. 다른 수정을 모두 이 테스트로 검증해야 한다 |
| 2 | M3, M4 | 실물에 처음 올리기 전에 막아야 하는 결함. 둘 다 고치는 양이 작다 |
| 3 | `cho_vla_core` 공통 (S5 외) | 두 VLA 호스트가 같이 쓰는 코어. 한 번 고치면 양쪽에 다 들어간다 |
| 4 | Franka VLA (S7, S11, S17, S12 외) | `cho_controller_franka` 를 CI 에 넣은 뒤(ST-1)에 하면 검증까지 받는다 |
| 5 | OpenArm (S14, S18, S6 외) | 실물 측정이 필요한 것이 많다 |
| 6 | 나머지 (S9, S20, S10 외) | 서로 독립적이다. 아무 때나 해도 된다 |
| 7 | 문서 정정 | 해당 코드 수정과 같은 커밋에 넣는다. 문서만 고치는 것은 따로 모은다 |
| 8 | 재발 방지 (ST-*) | 1단계가 끝난 뒤 순서대로 |

빌드는 패키지를 골라서 한다. 예: `cbp cho_vla_core cho_controller_openarm_mit`. 병렬도는 올리지 않는다(CLAUDE.md "Build Alias").

---

## 1단계 — M1: CI 복구

**문제.** `cho_bringup_franka` 는 `cho_controller_franka` 를, `cho_bringup_openarm` 은 `cho_hardware_openarm_mit_{mujoco,real}` 을 exec_depend 한다. 이 셋은 소스 트리에 있지만 CI 가 빌드하지 않는다. `ROSDEP_SKIP_KEYS` 는 rosdep 만 건너뛸 뿐이고, colcon 은 "작업공간에 있는데 선택되지 않았고 설치돼 있지도 않은 의존" 을 오류로 본다. 그래서 build 단계가 실패하고 테스트가 0개 돈다.

**수정** (`.github/workflows/ci.yml`)

1. 빌드하지 않는 소스 트리 패키지를 env 하나로 모은다.
   ```yaml
   # In the source tree but not built here (Franka's source-only deps, real
   # hardware). Ignored, not just skipped by rosdep: colcon refuses a workspace
   # dependency that is neither selected nor installed.
   NOT_BUILT_PACKAGES: >-
     cho_controller_franka
     cho_hardware_openarm_mit_mujoco
     cho_hardware_openarm_mit_real
   ```
   `ROSDEP_SKIP_KEYS` 에서 이 세 이름은 빼지 않는다. `rosdep --ignore-src` 가 여전히 알 수 없는 키로 본다.
2. Build 단계에 `--packages-ignore ${{ env.NOT_BUILT_PACKAGES }}` 를 붙인다. Test 단계의 `colcon test` 에도 붙인다.
3. 테스트가 0개면 실패하게 한다. `Test results` 단계(`if: always()`)를 다음처럼 바꾼다.
   ```bash
   set -o pipefail
   colcon test-result --verbose | tee /tmp/test_result.txt
   summary=$(tail -n 1 /tmp/test_result.txt)
   case "$summary" in
     "Summary: 0 tests"*) echo "::error::no test ran"; exit 1 ;;
   esac
   ```
4. `CLAUDE.md` 의 CI 문단(현재 56–62행 근처)을 고친다. 지금 문장은 "목록에 넣거나 ROSDEP_SKIP_KEYS 에 넣어라" 인데, 그것으로는 colcon 검사를 통과하지 못한다. 대체 문안:
   > A workspace package that a listed package depends on must either be listed too, or be named in NOT_BUILT_PACKAGES (colcon `--packages-ignore`) **and** ROSDEP_SKIP_KEYS. Only keys that are not in the source tree at all go in ROSDEP_SKIP_KEYS alone. The `rosdep --simulate` check does not catch a missing ignore; the build step does.

**검증**
- 로컬에서는 재현되지 않는다. `~/ros2_ws/install` 에 세 패키지가 이미 있기 때문이다.
- 브랜치에서 `workflow_dispatch` 를 한 번 돌려 확인한다. 기대하는 결과:
  - Build 가 통과한다.
  - 테스트 수가 직전 정상 run(087f4da, 1864개) 수준으로 돌아온다.
- 087f4da 에서 이미 1 failure 가 있었다. 그 실패가 다시 보이면 별도 항목으로 고친다. 0-test 가드와 섞지 않는다.

---

## 2단계 — 실물 투입 전 must-fix

### M3. OpenArm 실물 양팔: joint 청크를 위치 루프 없이 내보낸다

**확인한 사실**
- `drive_side_impedance: false` 는 `config/real/controllers_mit_bimanual.yaml` 에만 있다(104, 262, 424, 616행).
- MuJoCo 설정과 실물 단일 팔(`controllers_mit.yaml`)은 모두 `true` 다. 그래서 아래 거부 조건은 지금 검증된 MuJoCo 경로를 바꾸지 않는다.
- `task_space_impedance_controller.cpp:1476-1478`: `task_command && !drive_side_impedance_` 이면 `TRACKING_DAMPED_TORQUE` 가 된다. 이 모드는 q_des=측정값, kp=0 으로 나간다.

**수정**
1. `VlaController::build_chunk()`(`cho_controller_openarm_mit/src/vla_controller.cpp:555~`)에서, `parse_action_space` 바로 뒤에 거부 조건을 넣는다(fail-closed).
   ```cpp
   if (chunk.space == cho_vla_core::ActionSpace::kJoint && !drive_side_impedance_) {
     reason = "joint chunks need drive_side_impedance: true (the legacy law sends "
              "kp = 0 with q_des pinned to the measurement, so nothing closes the "
              "position loop)";
     return false;
   }
   ```
   기존 호출부(`:640-641`)가 `chunks_rejected` 를 세고 사유를 남긴다.
2. `on_configure` 에서 `!drive_side_impedance_` 이면 WARN 을 한 번 남긴다. 이 제어기는 task 청크만 받는다는 내용이다. 실패 원인이 launch 시점에 드러나게 하려는 것이다.
3. IMPEDANCE 모드로 바꾸는 방식은 쓰지 않는다. 이 설정은 `kp` 가 0×7 이라 IMPEDANCE 로 바꿔도 강성이 생기지 않는다.

**테스트.** `test_vla_controller.cpp` 에 `drive_side_impedance: false`, `kp` 0 fixture 를 추가한다.
- joint 청크 → `chunks_rejected == 1`, 기준이 움직이지 않음.
- task 청크 → 수락됨.

**문서.** 코드에 맞춰 고친다.
- `CLAUDE.md:93-94`
- `cho_robot_config/config/openarm.yaml:41-43`
- `controllers_mit.yaml:389-392`
- `launch/bringup_real_robot.launch.py:53-61` 의 인자 설명. 지금 경고하는 이유(관절 한계 스프링 누락)를 실제 이유(위치 강성 0)로 바꾼다.

### M4. VLA 상대 task 액션의 합성 프레임이 계약에 없다

**확인한 사실**
- `chunk_validator.cpp:160-167` 은 앵커 오른쪽에 곱한다. 즉 EE(body) 프레임 합성이다.
  ```
  from_anchor:  p = p_a + R_a·Δp,  R = R_a·ΔR
  per_step:     T_k = T_{k−1}·ΔT_k
  ```
- `ActionChunk.msg:19-24` 와 `types.hpp:46-54` 는 이 프레임을 적지 않는다. 대신 `per_step` 의 예로 "OpenVLA-OFT LIBERO EE deltas" 를 든다. 이 델타는 base 프레임 규약이다.
  ```
  p = p + Δp,  R = ΔR·R
  ```
- `vla_action_client.py:165` 는 `frame_id="base_link"` 를 붙여 relative 청크를 보낸다. 하지만 호스트는 `frame_id` 를 읽지 않는다.

**수정.** 메모리 규칙("새 필드는 선택, 빈 값이면 현재 동작 유지")에 맞춘다.
1. `ActionChunk.msg` 에 필드를 추가한다.
   ```
   # Frame a relative TASK-space delta is expressed in. Ignored for joint space
   # and for "absolute".
   #   "ee"   - body frame of the anchor: p = p_a + R_a·Δp, R = R_a·ΔR
   #   "base" - the model root frame:     p = p_a + Δp,     R = ΔR·R_a
   # Empty = "ee", the pre-v3 behaviour. An unrecognised value is REJECTED.
   string relative_frame
   ```
   per_step 이면 위 식을 이전 waypoint 에 대해 단계마다 적용한다.
2. `cho_vla_core`
   - `types.hpp` 에 `enum class RelativeFrame {kEe, kBase}` 와 `parse_relative_frame()` 을 둔다(빈 값 → `kEe`, 모르는 값 → false).
   - `Chunk` 에 필드를 추가한다.
   - `chunk_validator.cpp` 에서 `kBase` 일 때 왼쪽 곱으로 합성한다. 회전은 `ΔR·R`, 병진은 `p + Δp`.
3. 두 호스트(Franka `vla_action_server.cpp`, OpenArm `vla_controller.cpp::build_chunk`)가 새 필드를 파싱한다. 모르는 값은 거부한다.
4. 상대 task 청크가 `relative_frame` 없이 오면 WARN 을 throttle 해서 남긴다. 거부는 하지 않는다. 외부 브리지 4개가 바로 깨지지 않게 하려는 것이다.
5. 문서와 클라이언트를 고친다.
   - `ActionChunk.msg` 와 `types.hpp` 의 per_step 주석에서 LIBERO 예에 "`relative_frame: base` 로 보낼 것" 을 적는다.
   - `todo/VLA_TODO.md:282, 289` 의 매핑 표에 프레임 열을 추가한다.
   - `vla_action_client.py` 는 의도대로 `relative_frame="base"` 를 설정한다. 원 궤적이 지금과 거울상이 되는데, 그 방향이 원래 의도한 것이다.

**테스트** (`test_chunk_validator.cpp`). 기존 테스트 `TaskRelativeFromAnchorComposesInTheAnchorFrame` 는 Rz(90°) 앵커라 반전이 드러나지 않는다.
- 앵커 R_a = Rx(π), p_a = (0, 0, 0.4) 에서 per_step Δz = −0.01 × 8 스텝.
  - `ee` 이면 z = 0.48 이어야 한다(현재 동작 고정).
  - `base` 이면 z = 0.32 여야 한다.
- 같은 앵커에서 from_anchor 의 회전을 확인한다.
  - `ee` 이면 R = R_a·ΔR.
  - `base` 이면 R = ΔR·R_a.
- 모르는 `relative_frame` 값은 거부된다.

**참고.** 메시지를 바꾸므로 외부 브리지 4개를 다시 빌드해야 한다. 필드를 추가만 했으므로 지금 보내는 내용은 그대로 해석된다.

---

## 3단계 — `cho_vla_core` 공통

### S5. 블렌드 창 안에서 splice 하면 기준이 계단으로 뛴다
- 원인: `action_buffer.cpp:336-340`. 블렌드 중에 새 청크가 오면 `timeline_.previous = timeline_.waypoints;` 가 블렌드 전 타임라인으로 덮어쓴다. 그래서 블렌드가 처음부터 다시 시작한다.
- 수정:
  - `now < timeline_.blend_end` 이면, 지금 내보내는 블렌드 결과를 남은 waypoint 시각마다 샘플링한다. 그 결과로 `previous` 를 만든다.
  - splice 는 non-RT 쪽(RT 는 `readFromRT()` 로 읽는다)이므로 할당해도 된다.
- 회전 채널에 각가속도 상한을 둔다.
  - `ReferenceLimiter::Params` 에 `max_angular_acceleration` 을 추가한다. 기본값 0 이면 지금처럼 끈다.
  - 지금은 회전에 속도 clamp 만 있다(`reference_limiter.cpp:155-166`).
- 테스트(`test_action_buffer.cpp`): A(0)→B(1)→C(1), 간격 T = 1/15, τ = 0.1·0.2 로 연속 splice 한다. 출력의 한 주기 변화량이 블렌드 없는 경우를 넘지 않음을 확인한다.
- 문서: `DESIGN.md:138-142` 의 "neither a velocity nor an acceleration step" 이 고친 뒤에도 참인지 확인하고 문장을 맞춘다.

### splice 의 지난 waypoint 누적 (감사 관찰 · R0 logic-1)
- 원인: `action_buffer.cpp:330-334` 는 `t < new_start` 인 waypoint 를 모두 남긴다. 그래서 타임라인이 청크마다 늘고, splice 마다 전체를 복사한다. 600 s 에 약 36k 개다.
- 수정: 보간에 필요한 `now` 이전의 마지막 waypoint 하나만 남기고, 그보다 오래된 것은 버린다. `previous` 에도 같은 규칙을 적용한다.
- 테스트: 15 Hz × 10 분을 흉내 내 `timeline_.waypoints.size()` 가 청크 크기 + 상수 이하임을 확인한다.

### C11. 시간이 흐르지 않은 주기에 리미터가 1 ms 를 지어낸다
- `reference_limiter.cpp:84`: step ≤ 0 이면 직전 출력을 그대로 돌려준다(전진 0).
- 영향을 받는 곳은 Isaac 처럼 같은 state 가 반복되는 주기다(CLAUDE.md 의 `period == 0` 사례).

### C12. 한 goal 안에서 joint→task 로 바뀔 때 리미터를 다시 시드하지 않는다
- splice 결과의 `space_reset` 을 두 호스트 모두 읽지 않는다.
- 수정: `space_reset` 이면 지금 명령 중인 값(Franka 는 `H_ee_des`·`q_arm_des`, OpenArm 은 `idle_pose_`)으로 `limiter_.seed()` 한다. hold 에서 재개할 때의 시드 방식(`vla_action_server.cpp:648-660`)을 그대로 쓴다.

---

## 4단계 — Franka VLA

ST-1(`cho_controller_franka` 를 CI 에 넣기)을 먼저 하면, 아래 수정이 모두 `test_reactivation` 과 gtest 로 검증된다.

### S7. velocity 모드에서 비활성화해도 명령이 0 이 되지 않는다
`vla_controller.hpp` 에는 `on_deactivate` 가 없다. `JointSpaceVelocityController::on_deactivate` 와 같은 모양으로 추가한다.
```cpp
CallbackReturn VLAController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    // A velocity interface keeps its last value after the controller lets go, and a
    // simulator integrates it: leave the arm commanded to rest.
    if (control_mode_ == ControlMode::kVelocity) {
        for (auto & command : command_interfaces_) { command.set_value(0.0); }
    }
    return FrankaBaseController::on_deactivate(previous_state);
}
```
- 그리퍼 인터페이스가 같은 벡터에 있으면 팔 관절만 0 으로 쓴다. 쓰기 전에 인터페이스 구성을 확인한다.
- `test_reactivation` 에 VLA velocity 모드 케이스를 추가한다.
- ST-7(공용 헬퍼)로 가면 이 코드는 헬퍼 호출 한 줄이 된다.

### S11. 정책 기준이 관절 위치 한계 안에 묶이지 않는다
1. `vla_action_server.cpp:48-58` 의 한계기 설정에서 관절 창을 모델 한계로 채운다.
   - 창은 `q_lower_limits_ + margin` ~ `q_upper_limits_ − margin`, margin 은 파라미터로 두고 기본 0.05 rad 정도.
   - `clamp_joint_window = true` 로 둔다. OpenArm 호스트가 이미 이렇게 한다.
2. `write_open_loop()`(`vla_controller.cpp:304-313` 근처)에서 적분한 `q_ref_` 에 `clamp_to_joint_limits(q_ref_)` 를 적용한다. `task_space_ik_controller.cpp:170` 과 같은 패턴이다.
3. 테스트
   - 한계 밖 joint 청크 → 기준이 창 안에서 멈춘다.
   - 도달할 수 없는 task 목표(1.1 m) → 어느 관절도 하드 한계를 넘지 않는다.

### S17. hold 에 들어갈 때 기준 속도가 한 주기에 0 이 된다
- 원인: `vla_action_server.cpp:634-643` 의 hold 분기가 `latch_hold → apply_hold → v_arm_des.setZero()` 로 `limiter_.apply()` 를 건너뛴다.
- 수정
  - hold 에 들어가면 래치한 자세를 목표로 하고 **목표 속도 0** 인 기준을 만든다. 그 기준도 `limiter_.apply()` 를 통과시켜 Ruckig 가 감속하게 한다.
  - 래치는 "선 곳" 으로 한다. 즉 리미터가 멈춘 위치를 hold 목표로 갱신한다.
  - 지금 주석(gravity sag 를 따라가지 않게 frozen target 을 쓴다)의 의도는 그대로 지킨다. 측정값이 아니라 리미터 출력에서 래치하면 된다.
- 테스트: 0.1 rad/s 로 움직이는 중에 스트림이 끊긴다. 명령 가속이 `max_joint_ref_acc` 를 넘지 않음을 확인한다.

### S12. Isaac 250 Hz 에서 position/velocity 모드의 task IK 스텝이 발산한다
- 원인: `task_step()` 이 토크 모드 강성 `kp_task`(565 N/m)를 IK 이득 [1/s] 으로 그대로 쓴다. 그래서 K·dt = 565 × 0.004 = 2.26 > 2 다.
- 수정
  1. IK 전용 이득을 분리한다. 새 파라미터 `task_ik_gain` [1/s], 기본값은 1 kHz 와 250 Hz 모두에서 안정한 값(예: 50)으로 둔다.
  2. 스텝을 `Δq = J⁺(v_des·dt + K·e·dt)` 로 바꾸고, `limit_step()` 하나로 비례 축소한다.
  3. 첫 `update()` 에서 `nominal_period(period)` 를 알게 되면 `K·dt < 1` 을 검사한다. `get_update_rate()` 는 0 이라 configure 시점에는 알 수 없다. 위반하면 ERROR 를 남기고 hold 한다.
- 테스트: 1자유도 모형으로 dt = 0.004, 0.001 에서 수렴하는지 확인한다.

### 같은 호스트의 작은 항목
| ID | 수정 |
|---|---|
| S3 | `on_activate` 에서 `ReferenceHistory` 를 reset 한다(OpenArm `vla_controller.cpp:298` 과 같게). `reference_history.hpp:52-57` 의 fallback 문장을 코드에 맞춘다 |
| S8 | observation 시간원이면 t_obs 를 검사한다. 미래(`t_obs > now + ε`)이거나 너무 오래된(`now − t_obs > max_obs_age`) 청크는 거부한다. `admitted == 0` 인 청크는 watchdog 을 갱신하지 않는다 |
| C16 | `control_dt ≤ 0` 이면 지어내지 말고 거부한다. 두 호스트 모두 해당한다(OpenArm `build_chunk` 에도 같은 폴백이 있다). 메시지 주석도 "rejected" 로 고친다 |
| C17 | `goal_timeout_sec` 기본값을 0(끔)으로 해 DESIGN.md·OpenArm 과 맞춘다. 실물 Franka 동작이 바뀌므로 커밋 메시지에 적는다 |
| C23 | S8 을 넣은 뒤 `chunk_time_source` 기본값을 다시 정한다. 지금은 바꾸지 않는다 |
| C19 | TaskSpaceImpedance 와 VLA 토크-task 분기에 null-space 감쇠 `−(I − J⁺J)·D_null·dq` 를 추가한다(기본 작게, 실물 설정에서 켠다) |

---

## 5단계 — OpenArm

### S14. VLA task 경로의 명령 앵커에 anti-windup 이 없다
- 원인: 앵커 링에 포화 전 한계기 출력(`idle_pose_`, `vla_controller.cpp:753-755`, `:936`)이 들어간다. 그래서 막힌 곳 너머로 "더 누르라" 는 상대 청크가 계속 쌓인다.
- 수정
  - `joint_reference_offset()` 이 포화 스케일 s ∈ [0, 1] 를 돌려주게 한다.
  - 앵커 링에는 실제로 내보낸 값 `meas ⊕ s·(cmd ⊖ meas)` 를 넣는다.
  - legacy 법칙은 렌치 스케일로 같은 식을 쓴다.
- **Q1 답에 따라 범위가 달라진다**(아래 "결정이 필요한 것").
- 테스트: 기구학 벽 플랜트에서 5 s 누른 뒤 반전한다. 해제 지연이 측정 앵커 수준(약 340 ms)에 가까운지 확인한다.

### S18. drive-side 임피던스에서 오차가 null space 로 샌다
- 수정: 실물 `controllers_mit.yaml` 에 `reference_offset_weight` 를 kp/geomean(kp) 로 넣는다.
  - kp = [32, 30, 50, 49, 15, 10, 5] 이면 geomean ≈ 21.4, weight ≈ [1.50, 1.40, 2.34, 2.29, 0.70, 0.47, 0.23].
  - 넣기 전에 다시 계산한다.
- 강성 주석을 (J K⁻¹ Jᵀ)⁻¹ 기준으로 다시 쓴다.
- 검증: MuJoCo 에서 TCP 에 2 N·5 N 하중을 걸어 null 방향 변위를 잰다. 그다음 실물에서 같은 측정을 한다.

### S6. 실물 both_independent: 한 팔을 재활성화하면 다른 팔의 watchdog 이 트립한다
- 원인: 한 팔의 `on_activate` 가 enable 에서 100 ms 를 자는 동안, ResourceManager 락이 CM 루프를 막는다.
- 수정 후보(권장 순서)
  1. enable 대기를 `on_activate` 밖으로 뺀다. `on_activate` 는 enable 명령만 보내고, 이어지는 `read()` 주기들에서 응답을 확인하는 상태 기계로 바꾼다.
  2. (1)이 어렵다면, 다른 컴포넌트의 lifecycle 전환 중에는 watchdog 경과 시간을 멈춘다.
  3. 최소한 `docs/openarm_real_bringup.md:165` 에 "한 팔을 복구하기 전에 다른 팔을 hold 로" 절차를 적는다.
- 검증: 가짜 CAN 재현(리뷰 scratch)을 gtest 로 옮긴다. 왼팔 fresh 재활성화 동안 오른팔이 트립하지 않음을 확인한다.

### 같은 영역의 작은 항목
| ID | 수정 |
|---|---|
| S4 | `OpenArmBaseController::on_deactivate` 가 held-command ledger 에 기록하게 한다(`release_held_command`). ST-7 로 가면 같이 닫힌다 |
| S13 | 실물 MIT 소비자에 gain/τ_ff slew 와 최종 토크 포화를 추가한다. `docs/openarm_mit_contract_v1.md:269-279` 와 `:283-284` 의 모순을 먼저 정리한다 |
| S19 | `controllers_mit.yaml:116-130` 주석과 테스트를 실제 힘 상한(파생 offset)에 맞춘다. VLA 블록 주석의 kp·offset 산술을 다시 계산한다 |
| C21 | JointSpaceImpedance 에 궤적 가속 피드포워드(M·q̈_des)를 추가하거나, 주석의 폐루프 식을 지금 법칙에 맞춘다 |
| 문서 | `docs/openarm_real_bringup.md:198` 의 `mit_expected_update_rate_hz` 200 을 750 으로 고친다(launch 기본값·실물 프로파일과 맞춘다) |

---

## 6단계 — 나머지

### S9. MoveIt 브리지의 준비 질의가 응답을 잃으면 영구히 멈춘다
- 원인: `moveit_action_bridge.py:330-339`. `_ready_query_pending` 은 응답 콜백에서만 풀리는데, `call_async` 에는 timeout 이 없다.
- 수정
  - 질의를 시작할 때 `self._ready_query_started_at = time.monotonic()` 과 future 를 저장한다.
  - `_poll_ready` 에서 pending 이 `ready_query_timeout_sec`(기본 3 s)를 넘으면, 체인의 각 future 를 해당 client 의 `remove_pending_request(future)` 로 걷어 낸다. 그다음 pending 을 풀고 WARN 을 남긴다.
  - `base_service_behavior.py` 가 같은 방식을 쓴다.
- 테스트: 응답하지 않는 Trigger 서버를 띄웠다가 죽이고 다시 띄운다. 브리지가 READY 로 돌아오는지 확인한다.

### S20. Franka TaskSpaceQP 가 특이점 근처에서 포화한다
- 수정 방향(택일)
  - (a) SE3 태스크를 level 1 가중 비용으로 내리고, posture 를 작은 가중치로 둔다(감쇠 최소제곱).
  - (b) level 0 에 가속도·토크 bound 를 넣는다.
- 어느 쪽이든 `|ddq|` 상한 검사를 둔다.
- `CLAUDE.md:315` 의 "contact-aware force control" 을 고친다. 지금 접촉 코드는 주석 처리돼 있다.
- 동작이 바뀌는 수정이므로 MuJoCo 에서 기존 프리셋 추종 오차를 전후로 비교한다.

### S10. cho_object_pose 가 영상 시계 되감기에 대응하지 않는다
- `node.py:466-471`: 새 샘플의 stamp 가 창의 최신 stamp 보다 일정 이상(예: 1 s) 이르면 시간이 되감긴 것으로 본다. 그때 창을 비우고 INFO 를 남긴다. tf2 buffer 가 시간 역행을 처리하는 방식과 같다.
- 판정 로직은 ROS 없는 `geometry.py` 또는 `visibility.py` 쪽에 두고 pytest 로 확인한다. 리뷰 scratch 의 `repro_window_rewind.py` 를 테스트로 옮긴다.

### 작은 항목
| ID | 수정 |
|---|---|
| C13 | `hansung_scale_driver/device.py:339-345` 에서 재연결 close 와 서비스 write 를 같은 lock 으로 묶는다 |
| C14 | 재configure 때 옛 서버의 타이머를 먼저 cancel 하고 교체한다. 또는 콜백이 `weak_ptr` 을 잡게 한다(`goal_phase_action_server.hpp:139-152`) |
| C15 | `abort_active_goal` 이 문서대로 `H_ee_init` 을 되돌리게 하거나, 문서를 코드에 맞춘다(`task_space_server.hpp:209-217`) |
| C20 | TSQP/JSQP/OSC 의 하드코딩 "practical terms" 를 파라미터로 빼고, `rotor_inertia` 와 이중으로 계산되는지 확인한다 |
| C22 | Gazebo 손 중력 보상 g = 1.0 m/s² 의 근거를 주석에 적거나 파라미터로 뺀다 |
| C9 | FR5 trajectory replay 가 `motion_limit_parameters()` 를 쓰게 한다 |
| C18 | ActionChunk `seq` 순서 검사와 OpenArm `enable_gripper` 를 구현하거나, 계약에서 "미구현" 으로 표시한다 |

---

## 7단계 — 문서 정정 (코드 변경 없음)

| ID | 위치 | 고칠 내용 |
|---|---|---|
| C1 | `CLAUDE.md:81`, `:195`, `:93-94` | OpenArm 은 "simulation only" 가 아니다. 실물 bringup 과 `REAL_MIT_DIRECT_CONTROLLERS` 가 있다는 사실에 맞춘다 |
| C2 | `cho_interfaces/CONTRACT.md:213` | OpenArm MIT 제어기는 `~/controller_state`·`~/ee_state` 를 내지 않는다는 예외를 적는다 |
| C3 | `CLAUDE.md` 의 제어 주기 규칙 | gripper(raw period + `1e-3` 리터럴)를 고치거나 예외로 적는다. OpenArm MIT 의 누적 시계도 적는다 |
| C5 | `CLAUDE.md:541` | "controller classes in snake_case" → 실제는 PascalCase 클래스, snake_case 플러그인 이름 |
| C10 | `cho_vla_core/DESIGN.md:14-17` | 패키지 분리 근거를 지금 package.xml 에 맞춘다 |
| C6 | `ci.yml:91-94` 주석 | `cho_description_ur` 의 pytest 2개를 TEST_PACKAGES 에 넣거나, 주석을 고친다 |
| C8 | `cho_robot_config/config/openarm.yaml:205-231` | 'both' 프로필이 접두어 없는 역할 이름을 물려받는다. 역할을 재정의한다(코드가 아닌 설정 수정) |
| — | `CLAUDE.md` "Build Alias" | 리뷰 머신은 46 GB / 10코어로 보고됐다. 6 GB 문구가 다른 머신 기준이라면 그렇게 적는다. 제한 값은 바꾸지 않는다 |

---

## 8단계 — 재발 방지

| ID | 내용 | 첫 단계 |
|---|---|---|
| ST-1 | `cho_controller_franka` 를 CI 빌드·테스트에 넣는다. Franka 쪽 의존은 `franka_msgs` 하나이고 CI 가 deb 을 이미 설치한다 | 1단계로 초록 기준선을 확인한 뒤 별도 커밋으로 넣고 `workflow_dispatch` |
| ST-2 | CI 패키지 선택을 opt-in 목록에서 opt-out(`NOT_BUILT_PACKAGES`)으로 바꾼다 | `colcon list --names-only --packages-ignore $NOT_BUILT` 결과와 지금 목록을 비교 |
| ST-3 | M4 의 비대칭 앵커 테스트에 euler·rotation6d 비대칭 디코드 테스트를 더한다(S16) | M4 PR 에 포함 |
| ST-4 | launch golden `expected/*.txt` 의 spawner 인자와 레지스트리 이름을 대조하는 테스트(C7, C8) | 단언 없이 "spawn 되지 않은 레지스트리 이름" 표를 한 번 출력 |
| ST-5 | OpenArm 양팔 YAML 좌우 대칭 테스트를 6개 파일 전 블록으로 넓힌다 | mujoco/isaac 양팔 파일에 먼저 적용 |
| ST-7 | 비활성화 의무(velocity→0, position→ledger)를 `cho_controller_base` 헬퍼 하나로 옮긴다(S7, S4) | 헬퍼와 gtest, UR 에 먼저 적용 |
| ST-8 | 두 VLA 호스트의 결선·수명을 `cho_vla_core` 의 ROS 없는 세션으로 옮긴다(S3, S8, S11, S17, C17) | 4단계 직접 수정을 먼저 넣은 뒤 |
| ST-10 | 실물 OpenArm MIT 경로를 CI 에 넣는다(S6, S13) | `openarm_can` 의 CLI11 의존 선언부터 |
| ST-11 | MuJoCo limiter 와 실물 ArmConsumer 의 튜플 정형 차이를 차분 gtest 로 잰다 | SEEDING kp 3→32 계단 시퀀스 gtest 1개 |

리뷰가 비추천한 것(ST-12~17)은 하지 않는다.

---

## 결정이 필요한 것

| # | 질문 | 영향 |
|---|---|---|
| Q1 | 상대 청크의 앵커는 학습 데이터 규약상 **측정** 자세인가, **명령** 자세인가? | 명령이면 S14 의 anti-windup 만으로 닫힌다. 측정이면 OpenArm task 경로와 Franka position/velocity 의 앵커 선택 자체를 바꿔야 한다 |
| S15 | 외부 브리지 4개가 goal 의 `stream_timeout` 을 설정하는가? | 설정하지 않으면, 재추론 간격 0.27–0.8 s 에서 0.2 s watchdog 이 지평선이 남은 채로 hold 에 들어간다. 그 경우 기본값과 문서를 보완한다 |
| S16 | euler·rotation6d 를 보내는 브리지가 있는가? | 있으면 M4 와 같은 방식으로 `ActionChunk.msg` 에 규약(합성 순서·단위·행/열)을 적는다. GR00T 6D 는 행 규약이라 지금 디코더로 읽으면 Rᵀ 가 된다 |
| — | 추적되지 않는 `extern/qpOASES` 를 지워도 되는가? | 남겨 두면 패키지를 고르지 않는 빌드(`cbr`)에 qpOASES 가 끌려 들어간다. 지우거나 `COLCON_IGNORE` 를 둔다 |
| — | 3–6개월 안에 OpenArm 에 양팔 단일 정책을 붙이는가? | 붙인다면 팔별 독립 생존 판정(5789831)을 다시 볼지 정한다 |
| — | flake8 줄 길이 기준은 99열인가, 120열인가? | CLAUDE.md 는 120 이다. 다른 곳에 99 가 있으면 한쪽으로 맞춘다 |

## 실물·시뮬레이터에서만 확인되는 것
- M3·S18: 실물 마찰 아래의 크기
- S6: 실물 팔에서 watchdog 트립 후 낙하 여부
- S12: Isaac 물리에서의 반응
- S17: FR3 실기에서 reflex 가 실제로 나는지
- M1: 고친 뒤 실제 CI 통과
