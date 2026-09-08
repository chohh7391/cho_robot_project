# 플래닝 시간 기준선 — OMPL (Step 0)

`todo/CUROBO_MOVEIT_TODO.md` Step 0의 산출물. **목적은 "cuRobo를 붙이면 계획이 빨라지는가"를
붙이기 전에 판정하는 것**이었고, 결과는 **붙일 이유가 없다**로 나왔다.

## 무엇을 어떻게 재는가

`bench_planning.py`는 `cho_moveit_common/scripts/moveit_action_bridge.py`가 만드는 것과
**같은 형태의 MoveGroup 요청**을 보낸다 (그룹, 파이프라인, `num_planning_attempts=5`,
속도/가속 스케일 0.25, 위치 허용오차 5 mm, 방향 허용오차 0.01 rad). 다른 점은 두 가지뿐이다:

- `planning_options.plan_only = True` — 로봇은 움직이지 않는다. 계획만 잰다.
- `start_state`를 home1로 **명시 고정** — 시뮬레이터가 어디 서 있든 매 시행이 같은 문제를 푼다.

측정 항목은 `result.planning_time`(move_group이 보고하는 순수 계획 시간), 액션 왕복 시간,
그리고 `planned_trajectory`의 마지막 `time_from_start`(= 궤적 실행 예상 시간)이다.
마지막 항목이 있어야 "계획이 전체 사이클의 몇 %인가"를 계산할 수 있다.

목표 집합:

- **joint**: `cho_robot_config/config/fr5.yaml`의 `poses.home` 프리셋. `home_safety`에서
  `enabled: false`인 것(home0, 진단 전용)은 제외.
- **pose**: `motions.reach` 프리셋 4개 + 그 주변 상자에서 뽑은 랜덤 포즈 12개(시드 고정).
  랜덤 목표 중 5개(`rand00/06/07/09/11`)는 도달 불가로 어떤 조건에서도 실패한다.
  `summarize.py`가 이들을 따로 떼어 중앙값을 오염시키지 않게 한다.

씬:

- `floor` — 현행 static scene(바닥만). 실제 운용 조건.
- `floor_obstacle` — 바닥 + 케이지(기둥 2개 + 벽 1개 + 위쪽 슬래브).

## 재현

```bash
ros2 launch cho_bringup_fr5 bringup_gz_moveit.launch.py gazebo_gui:=false launch_rviz:=false
```

```bash
python3 todo/curobo_bench/bench_planning.py --robot fr5 --pipeline ompl --repeats 5 --out todo/curobo_bench/ompl_baseline.csv
```

```bash
python3 todo/curobo_bench/summarize.py todo/curobo_bench/*.csv
```

`--obstacle`은 `x,y,z,dx,dy,dz`를 `;`로 이어 여러 상자를 준다. 값이 `-`로 시작하므로
argparse가 옵션으로 오인하지 않도록 **`--obstacle=...` 등호 형식**으로 넘겨야 한다.

## 결과 (2026-09-08, FR5 / Gazebo / 190회 계획)

| 씬 | 목표 | 성공 | 계획 중앙값 | p95 | 최대 | 궤적 | 계획 비중 |
|---|---|---|---|---|---|---|---|
| floor | joint | 15/15 | 5.6 ms | 14.4 ms | 15.1 ms | 5.21 s | **0.11%** |
| floor | pose | 55/55 | 15.2 ms | 32.6 ms | 228.1 ms | 6.57 s | **0.23%** |
| floor_obstacle | joint | 5/15 | 12.6 ms | 22.0 ms | 22.0 ms | 0.00 s | (주1) |
| floor_obstacle | pose | 52/55 | 25.0 ms | 46.4 ms | 226.7 ms | 12.46 s | **0.20%** |

주1: 케이지 씬에서 성공한 joint 목표는 home1뿐이고, home1은 시작 자세와 같아 궤적 길이가 0이다.
따라서 이 행의 "계획 비중"은 무의미하다. home2/home3는 케이지와 충돌해 실패한다.

**실패 지연**: 도달 불가 목표는 두 씬 모두 **5.02 s**를 소진한다
(`allowed_planning_time=5.0`을 끝까지 쓴다). 이 데이터셋에서 가장 큰 숫자다.

장애물 배치를 세 번 바꿔 봤지만 계획 시간 중앙값은 움직이지 않았다:

| 배치 | pose 성공 | 계획 중앙값 | p95 |
|---|---|---|---|
| floor (장애물 없음) | 55/80 | 15.2 ms | 32.6 ms |
| cage | 52/80 | 25.0 ms | 46.4 ms |
| tight cage (1회 시행) | 8/16 | 19.3 ms | 243.3 ms |

장애물을 조이면 **성공률만 떨어진다** — 목표가 실현 불가능해질 뿐 계획이 어려워지지 않는다.
6축 팔에 대체로 열린 작업공간은 RRTConnect에게 쉬운 문제다.

## 판정

Step 0의 중단 기준은 "계획 시간 중앙값 < 50 ms **이고** 전체 사이클의 5% 미만"이었다.
실측은 **15~25 ms / 사이클의 0.2%**로, 기준을 25배 여유로 통과한다.
가장 느린 단일 계획(228 ms)조차 6.6 s 궤적의 3.5%다.

**cuMotion이 계획 시간을 0으로 만들어도 사이클은 0.2% 짧아진다.**
따라서 "플래닝을 빠르게"를 목적으로 하는 한 cuRobo 통합은 진행하지 않는다.

## 이 데이터가 대신 가리키는 것

1. **사이클을 지배하는 것은 궤적 길이다** (floor 6.6 s, 케이지 12.5 s). 그리고 이것은 플래너가
   아니라 `moveit.execution.max_velocity_scaling_factor: 0.25`가 정한다. 사이클 단축이
   목적이라면 지렛대는 여기다. cuMotion도 스케일링을 `time_dilation_factor`로 받으므로
   0.25를 그대로 주면 똑같이 4배 늘어진다.
2. **장애물 씬에서 궤적이 2배로 길어진다**(6.6 → 12.5 s). 계획 시간은 그대로인데 우회 경로가
   길어진 것이다. cuRobo의 궤적 최적화가 값을 낼 자리가 있다면 여기이고, 그렇다면 판정 기준을
   계획 시간이 아니라 **궤적 길이와 추종 품질**로 다시 써야 한다.
3. **실패 지연 5.02 s**는 `allowed_planning_time`을 줄이는 것만으로 대부분 해결된다.
   브리지는 이 값을 Cho 액션의 duration에서 받아 `max(1.0, min(duration, 10.0))`으로 쓴다.

## 함정 기록

- **장애물이 시작 자세와 겹치면 모든 계획이 플래너에 닿기도 전에 실패한다.** 첫 시도에서
  상자를 `upperarm_link` 위에 놓아 95/95 실패를 얻었다. move_group 로그에는
  `fix_start_state_collision: Unable to find a valid state nearby the start state`가 남고,
  반환 코드는 `FAILURE(99999)`여서 겉보기로는 타임아웃과 구별되지 않는다.
  그래서 `bench_planning.py`는 장애물을 올린 뒤 `/check_state_validity`로 시작 자세를
  검사하고, 충돌이면 측정을 거부한다.
- 도달 불가 목표를 중앙값에 섞으면 안 된다. `summarize.py`가 자동으로 분리한다.

---

# 속도 스케일링 스윕 (0.25 / 0.50 / 1.00)

위 결과가 "사이클을 지배하는 건 궤적 길이고 그건 스케일링이 정한다"를 가리켰으므로,
같은 하니스로 스케일링만 바꿔 재봤다. `floor` 씬, pose/joint 목표, 5회 시행,
`scaling_{0.25,0.50,1.00}.csv`. **plan-only이므로 실제로 그 속도로 움직인 것은 아니다.**

```bash
python3 todo/curobo_bench/bench_planning.py --robot fr5 --pipeline ompl \
    --repeats 5 --scenes floor --scaling 0.50 --out todo/curobo_bench/scaling_0.50.csv
```

## 결과

| 목표 | scale | 궤적 중앙값 | 0.25 대비 | 이론(가속 제한) | 이론(속도 제한) |
|---|---|---|---|---|---|
| joint | 0.25 | 5.21 s | 1.00x | 1.00x | 1.00x |
| joint | 0.50 | 3.66 s | **1.43x** | 1.41x | 2.00x |
| joint | 1.00 | 2.59 s | **2.02x** | 2.00x | 4.00x |
| pose | 0.25 | 7.98 s | 1.00x | 1.00x | 1.00x |
| pose | 0.50 | 4.76 s | **1.68x** | 1.41x | 2.00x |
| pose | 1.00 | 3.62 s | **2.20x** | 2.00x | 4.00x |

계획 시간은 어느 스케일에서도 13~23 ms로 변하지 않았다(계획 비중 0.2 → 0.5%).

## 해석: 스케일 4배가 궤적 2배밖에 못 줄인다

측정이 **가속 제한(삼각 프로파일) 이론과 거의 정확히 일치한다** — joint은 1.43x vs 1.41x,
2.02x vs 2.00x. MoveIt은 속도와 가속을 **같은 인자로 함께** 스케일하므로 램프 구간
`v_eff/a_eff = v/a = 2.25 s`가 스케일과 무관하게 고정된다. 그래서 4배 스케일이 4배 단축이 아니다.

궤적 시간을 `T = A/s + B`로 맞추면:

| 목표 | A | **B (스케일 무관 고정분)** |
|---|---|---|
| joint | 0.86 | **1.81 s** |
| pose | 1.47 | **2.01 s** |

즉 스케일을 무한히 올려도 **1.8~2.0 s 아래로는 내려가지 않는다.** 이 바닥을 낮추는 것은
스케일링이 아니라 `joint_limits.yaml`의 `max_acceleration: 0.7 rad/s²`다.

## 실기 제약 — 이 이득은 지금 쓸 수 없다

`cho_moveit_fr5/config/joint_limits.yaml`의 주석이 기록한 사실:
데이터시트 한계에 스케일 0.25를 준 **0.79 rad/s를 실기가 거부했다**
(`ServoJ refused, code 14 / ERR_EXECUTION_FAILED`). 그래서 한계를 데이터시트의 절반
(1.575 rad/s)으로 낮춰 커미셔닝 상한을 0.394 rad/s로 잡았다.

현재 설정에서 명령되는 속도:

| scale | v_eff | 상태 |
|---|---|---|
| 0.25 | 0.394 rad/s | 커미셔닝 상한, 동작 확인됨 |
| 0.50 | **0.7875 rad/s** | **거부된 0.79 rad/s와 동일한 값** |
| 1.00 | 1.575 rad/s | 거부값의 2배 |

따라서 위 표의 1.68x(0.50)는 **실기에서 현재 쓸 수 없는 이득**이다. 진짜 상한을 정하는 것은
MoveIt도 스케일링 팩터도 아니라 드라이버의 ServoJ 수용 한계다.
주석도 같은 말을 한다: "스케일링은 요청별 편의값이지 안전 한계가 아니다."

## 사이클 단축 지렛대 순위 (이 데이터 기준)

1. **`max_acceleration` (현재 0.7 rad/s²)** — 1.8~2.0 s의 바닥을 정하는 값. 구조적으로 가장 큰
   여지가 여기 있다. 단 올리려면 ServoJ 거부와 같은 실기 검증이 필요하다.
2. **속도 스케일링** — 최대 2배. 단 실기에서는 0.25~0.5 사이에서 막힌다(위 표).
3. **플래너 교체(cuRobo)** — 사이클의 0.2~0.5%. 무관하다. 같은 관절 한계 아래에서는
   cuRobo의 궤적도 더 빨리 갈 수 없다.

---

# cuMotion 대 OMPL (Step 3 판정)

연동은 완성됐고 동작한다. 아래는 **같은 목표 세트·같은 씬·같은 요청 형태**로 각 190회 측정한
결과다. 원자료 `ompl_baseline.csv` / `cumotion.csv`, 비교는
`python3 summarize.py ompl_baseline.csv cumotion.csv`.

cuMotion 쪽 설정: XRDF는 `cho_moveit_fr5/config/fr5.xrdf`, `velocity_scale 0.5`,
`num_trajopt_time_steps 64`, `time_dilation_factor`는 요청의 0.25.

| 씬 | 목표 | 파이프라인 | 성공 | 계획 중앙값 | p95 | 궤적 |
|---|---|---|---|---|---|---|
| floor | joint | OMPL | 15/15 | **5.6 ms** | 14.4 ms | **5.21 s** |
| floor | joint | cuMotion | 15/15 | 243.6 ms | 258.8 ms | 24.46 s |
| floor | pose | OMPL | 55/55 | **15.2 ms** | 32.6 ms | **6.57 s** |
| floor | pose | cuMotion | 55/55 | 243.7 ms | 272.7 ms | 12.33 s |
| 장애물 | pose | OMPL | 52/55 | 25.0 ms | 46.4 ms | 12.46 s |
| 장애물 | pose | cuMotion | **0/55** | — | — | — |

**cuMotion이 이긴 항목은 하나뿐이다 — 실패 포기 속도.**
도달 불가 목표에서 OMPL은 `allowed_planning_time` 5.02 s를 끝까지 소진하는데 cuMotion은
**0.11~0.29 s**에 포기한다. IK를 먼저 풀기 때문이다.

## 왜 궤적이 2~4.7배 긴가 — 균일 dt

`trajopt_tsteps`를 32~64로 쓸어보면 성공률과 궤적 길이가 **같은 노브에서 정반대로** 움직인다
(pose 4개 + home2/home3의 FK 포즈 = 6개 목표, 표준 설정):

| tsteps | 성공 | 궤적 중앙값 | 궤적 최대 | solve 중앙값 | 최대 가속 |
|---|---|---|---|---|---|
| 32 | 5/6 | 4.78 s | 8.27 s | 73.2 ms | 0.044 |
| 40 | 5/6 | 5.24 s | 8.19 s | 72.8 ms | 0.044 |
| **48** | **6/6** | 7.45 s | **23.74 s** | 70.0 ms | 0.044 |
| 56 | 6/6 | 7.74 s | 26.60 s | 75.8 ms | 0.044 |
| 64 | 6/6 | 8.91 s | 29.60 s | 73.5 ms | 0.044 |

성공에는 48 이상이 필요하고, 그 지점에서 최악 궤적이 8.27 → 23.74 s로 뛴다. **타협점이 없다.**

최대 가속이 모든 tsteps에서 0.044인 것이 원인을 알려준다. dilation 전 값은
`0.044 / 0.25^2 = 0.70`, 즉 **가속 한계를 정확히 포화**시키고 있다. 한계가 낮아서 느린 것이
아니다. cuRobo는 고정 웨이포인트 수에 **하나의 dt**를 최적화하므로 궤적 전체가 가장 빡빡한
구간의 속도로 눌리고, 길이는 `(N-1) x dt`로 웨이포인트 수에 따라 단조 증가한다. MoveIt의
TOTG는 구간마다 다른 시간을 줄 수 있어 쉬운 구간을 빠르게 지난다. 이 차이가 2~4.7배다.

## 장애물 씬 0/55 는 플래너 성능이 아니다 — 모델 불일치

플래너 노드 로그: `MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION`.
**cuRobo가 시작 자세를 충돌로 판정한다.** 같은 자세를 MoveIt의 `/check_state_validity`는
통과시킨다 (`bench_planning.py`가 측정 전에 검사하고, 통과하지 못하면 측정을 거부한다).

원인은 cuRobo가 로봇을 **콜리전 스피어**로, MoveIt이 **메시**로 보기 때문이다. 스피어는 팔을
부풀려 감싸므로 MoveIt이 여유롭다고 보는 간격을 cuRobo는 충돌로 본다. 이 케이지는 MoveIt
기준으로 배치를 골랐으니(§함정 기록) cuMotion에게는 처음부터 불가능한 씬이었다.

그래서 이 행은 **비교로 읽으면 안 된다.** 다만 벤치마크 결함으로만 볼 것도 아니다 —
운영상 실질 위험이다: **로봇이 장애물 근처에 서 있으면 OMPL은 계획하고 cuMotion은 거부한다.**
두 파이프라인이 같은 planning scene을 다르게 본다. 튜닝으로 덮을 문제가 아니고, 스피어를
메시에 맞게 다시 깎거나(로봇당 하루 규모) 두 모델의 간격을 운영 여유로 흡수해야 한다.

향후 이 씬을 다시 재려면 `bench_planning.py`의 시작 자세 검사를 MoveIt뿐 아니라
**cuRobo 모델로도** 해야 한다. 지금은 MoveIt만 본다.

## 계획 시간 244 ms 의 정정

**앞선 초안은 이 숫자를 "cuMotion 이 OMPL 보다 16~43배 느리다"로 적었다. 그것은 틀렸다.**
244 ms 는 cuRobo 의 시간이 아니다. 같은 설정에서 cuRobo 를 직접 호출해 재면:

| 항목 | 값 |
|---|---|
| `solve_time` | 40 ms |
| `total_time` (플래너 노드가 MoveIt 에 보고하는 값) | 51~77 ms |
| 파이썬 쪽 wall | 51~77 ms |

즉 **cuRobo 54 ms + 경로 오버헤드 190 ms** 다. 오버헤드는 플러그인이 planning scene 전체를
매 요청마다 `planning_scene_diff` 로 직렬화해 보내고, 파이썬 쪽이 그것으로 cuRobo 월드를
매번 재구축하는 비용이다. 플러그인의 폴링(`kSleepIntervalInMs = 5`)은 원인이 아니다.

"세 배"와 "열여섯 배"는 다른 이야기다. 프로젝트가 실제로 겪는 지연은 244 ms 가 맞지만
그것은 **플래너의 성질이 아니라 이 플러그인 구조의 비용**이고, 줄일 여지가 있는 항목이다.

또 하나 정정: 초안은 "cuRobo 는 고정 GPU 작업량이라 난이도와 무관하게 상수"라고 적었다.
정확하지 않다. **시행당** 비용이 거의 상수이고, `max_attempts` 안에서 재시도 횟수가 난이도에
따라 달라진다. 같은 floor 씬에서 목표 세트만 바꾸자 244 ms → 83 ms 로 떨어졌다.

## GPU 경합은 아니었다

측정 중 `nvidia-smi` 를 0.5 s 간격으로 샘플링했다: SM 클럭 2662~2917 MHz(부스트 근처,
스로틀 없음), solve 중 사용률 87% 까지 상승, 배경 10~19%. 타이밍도 재현된다.
배경 소비자는 rustdesk(NVENC)·Slack·브라우저로 SM 을 쓰지 않는다.

---

# 어려운 씬 — 좁은 통로에서는 cuRobo 가 이긴다

위 결과는 **쉬운 씬만 본 것이었다.** 공개 벤치마크(Black Coffee Robotics, UR10 3개 씬)는
cuMotion 0.134~0.301 s / 성공률 100%, RRTConnect 40~90% 를 보고한다. 우리 cuMotion 측정
(0.244 s)은 그 범위 안이다. **비정상인 쪽은 OMPL 15 ms 였다** — 우리 씬이 너무 쉬웠다.

## 씬 설계에 다섯 번 실패했다 (그 자체가 결과다)

| 시도 | 결과 | 이유 |
|---|---|---|
| 케이지(기둥+벽+슬래브) | 성공률만 하락, 시간 불변 | 통로를 만들지 않고 목표를 불가능하게 만듦 |
| 수직 통로 4종 | 시작 자세 충돌 | home1 의 엘보가 `(-0.30,-0.15,0.45)` — 벽 놓은 자리 |
| 창 뚫린 벽 (28/22/18 cm) | 0/5 | 목표가 벽 뒤 15 cm 면 손목만 아니라 엘보까지 통과해야 함 |
| 위 열린 통 (30~16 cm) | 0/5 | 목표 자세의 엘보 위치가 팔 길이(0.425 m)로 도달 불가 |
| 얇은 판 (경로 측정 후 배치) | **성공** | 우회만 강제, 경로를 없애지 않음 |

핵심 원인: **home1 은 팔이 작업공간 중앙에 접힌 자세**다(수평 반경 0.225 m, 엘보가 중앙 관통).
근처에 무엇을 놓아도 시작 자세와 충돌한다. `upright`(`[0,-90,0,0,-90,0]`)는 팔이 수평 반경
0.120 m 의 좁은 수직 기둥으로 모여서 나머지 공간이 열린다. 어려운 씬은 이 자세에서만 만들어졌다.

**그래서 이 프로젝트의 표준 시작 자세(home1)로는 OMPL 이 힘들어하는 씬을 구성할 수 없다.**
앞 장의 "장애물을 조여도 OMPL 이 15~25 ms 유지" 는 그 결과였다.

## 통과한 설계 — 경로를 측정해서 막는다

손으로 배치를 추측하는 대신: 장애물 없이 먼저 풀어 팔이 지나가는 곳을 얻고, 그 경로 중간에
**두께 3 cm** 판을 세운다. 얇게 하는 것이 핵심이다 — 부피가 크면 "불가능"이 되고 얇으면
"우회 필요"가 된다.

```
시작   upright = [0, -1.5708, 0, 0, -1.5708, 0]      (home1 이 아니다)
목표   (-0.25, -0.10, 0.45) 주변, 툴 z 는 -world z
판     중심 (-0.323, -0.092, 0.70), 0.50 x 0.50 x 0.03   <- 자유공간 해의 경로 중간 지점
```

cuRobo 단독 검증: free 5/5 궤적 20.74 s → 판 5/5 궤적 27.62 s (우회가 생겼고 풀린다).

## 결과 (각 72회, `hard_ompl.csv` / `hard_cumotion.csv`)

| 씬 | 목표 | 파이프라인 | 성공 | 계획 중앙값 | p95 | 궤적 |
|---|---|---|---|---|---|---|
| 판 없음 | pose | OMPL | 27/27 | 24.0 ms | 26.4 ms | 9.05 s |
| 판 없음 | pose | cuMotion | 27/27 | 83.0 ms | 88.0 ms | 21.39 s |
| **판** | **pose** | **OMPL** | **23/27** | 26.7 ms | 32.0 ms | 13.02 s |
| **판** | **pose** | **cuMotion** | **27/27** | 82.6 ms | 272.9 ms | 28.05 s |
| 판 | joint | OMPL | **3/9** | 18.4 ms | 27.2 ms | 11.30 s |
| 판 | joint | cuMotion | **0/9** | — | — | — |

## 기전 — 충돌을 인식하는 IK

목표별로 보면 분명하다:

```
rand01   OMPL 2/3   cuMotion 3/3
rand03   OMPL 2/3   cuMotion 3/3
rand05   OMPL 2/3   cuMotion 3/3
rand07   OMPL 2/3   cuMotion 3/3
home2    OMPL 3/3   cuMotion 0/3     <- 유일하게 OMPL 만 성공
```

OMPL 은 **같은 목표를 시행마다 놓친다**(2/3). 그리고 그 실패는 **0.02 s 에 끝난다** —
`allowed_planning_time` 을 쓰다 실패한 것이 아니라 목표 자세를 만들지 못해 즉시 거부다
(코드 `INVALID_MOTION_PLAN` / `FAILURE`).

목표가 판 아래에 있으면 6축 팔의 팔뚝이 판을 가로지르는 IK 해가 나온다. MoveIt 의 기본 IK 는
충돌을 모르므로 그 해를 내놓고 거부당하고, 무작위 재시작이라 시행마다 결과가 달라져 2/3 가 된다.
cuRobo 는 **충돌을 인식하는 IK 를 GPU 에서 병렬로** 풀어 판을 피하는 분기를 찾는다.

**즉 cuRobo 의 강점은 계획 속도가 아니라 도달 성공률이다.** 문헌의 인식과 일치한다 —
같은 공개 벤치마크도 성공률 차이를 보고하고, 계획 시간은 cuMotion 이 더 느렸다.

`home2` 만 방향이 반대인 것도 설명된다: joint 목표는 플래너 노드가 FK 로 포즈로 바꿔
`plan_single` 에 넘기므로 판을 피할 자유도가 사라진다.

## 판정 (정정판)

Step 3 의 원래 채택 기준(성공률 동등 이상 + 계획 시간 2배 개선 + 궤적 정상)은
**단일 기본 파이프라인을 고르는 기준**이었다. 측정은 그 질문이 잘못 세워졌다고 답한다:
승자가 목표 종류와 씬 난이도에 따라 갈린다.

| 조건 | 승자 | 근거 |
|---|---|---|
| 쉬운 씬(현행 운용 조건), 모든 목표 | **OMPL** | 15~25 ms 대 83~244 ms, 궤적 2~4.7배 짧음, 성공률 동등 |
| 좁은 씬, **task(pose) 목표** | **cuMotion** | 27/27 대 23/27. 충돌 인식 IK |
| **joint 목표** (씬 무관) | **OMPL** | 3/9 대 0/9. FK 변환이 자유도를 없앰 |
| 실패 포기 속도 | cuMotion | 0.11~0.29 s 대 5.02 s |

**이것이 정확히 `todo/CUROBO_MOVEIT_TODO.md` D1 의 배선이다** — task 목표만 cuMotion,
joint 목표는 OMPL 유지. 설계 당시에는 소스를 읽은 추론이었고, 이제 측정이 뒷받침한다.

**기본값은 OMPL 로 둔다**(D2). 현행 운용 조건이 앞 표의 첫 행이기 때문이다. cuMotion 은
좁은 씬을 다루게 될 때 `cumotion:=true` 로 켜는 opt-in 으로 남긴다. 켤 때 감수하는 것:
계획 지연 3배(그중 190 ms 는 플러그인 오버헤드), 궤적 길이 2배, 그리고 아래의 모델 불일치.

## 켜기 전에 반드시 해결해야 할 것 — 구/메시 불일치

앞 장의 케이지 0/55 는 성능이 아니라 모델 차이였다(`INVALID_START_STATE_WORLD_COLLISION`).
**MoveIt 이 통과시킨 시작 자세를 cuRobo 가 거부한다.** 운영에서는 이렇게 나타난다:
로봇이 장애물 근처에 서 있으면 OMPL 은 계획하고 cuMotion 은 거부한다.

`bench_planning.py` 의 시작 자세 검사는 MoveIt 메시 모델만 본다. cuRobo 쪽은 별도 스크립트로
확인해야 한다(이번 어려운 씬은 그렇게 양쪽 다 검증했다). 근본 해결은 스피어를 메시에 맞게
다시 깎는 것(로봇당 하루 규모)이거나 두 모델의 간격을 운영 여유로 흡수하는 것이다.

---

# 어려운 씬 재현

```bash
ros2 launch cho_bringup_fr5 bringup_gz_moveit.launch.py \
    gazebo_gui:=false launch_rviz:=false cumotion:=true
```

```bash
python3 todo/curobo_bench/bench_planning.py --robot fr5 --pipeline ompl \
    --repeats 3 --random-goals 8 --scenes floor,floor_obstacle --warmup 2 \
    --start-joints "0,-1.5707963268,0,0,-1.5707963268,0" \
    --goal-anchor=-0.25,-0.10,0.45 \
    --obstacle=-0.323,-0.092,0.70,0.50,0.50,0.03 \
    --out todo/curobo_bench/hard_ompl.csv
```

`--pipeline isaac_ros_cumotion` 으로 한 번 더 돌리고 `summarize.py` 로 비교한다.

**주의 — 값이 `-` 로 시작하는 인수는 등호 형식으로 넘겨야 한다**
(`--goal-anchor=-0.25,...`). argparse 가 옵션으로 오인한다.

**주의 — 이 측정의 시작 자세는 `home1` 이 아니다.** `upright` 다. 따라서 이 장의 숫자는
**로봇 능력 비교**이고 **이 프로젝트의 실제 운용 조건이 아니다.** 운용 조건(home1, 바닥만)
숫자는 앞 장에 있다. 두 장은 서로 다른 질문에 답하므로 섞어 인용하면 안 된다.
