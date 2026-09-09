# Session Log

## §0 지금 상태

**VLA 파이프라인이 로봇 무관 코어로 분리되고 OpenArm MIT에 VLA 컨트롤러가 붙었다.
테스트 438개 + MuJoCo 프로브 18/18 통과. joint/task 양쪽 다 실제로 추종한다.**

새 패키지 `cho_controller/utils/cho_vla_core`(구현 1615줄, gtest 87개)가 청크 검증·시간 정렬
splice·샘플링·레이트 제한·스트림 워치독을 소유한다. ROS 없이(rclcpp 미의존) 컴파일되므로
controller_manager 픽스처 없이 테스트된다. Franka `VLAActionServer`는 이 코어의 어댑터로
축소됐고, `cho_controller_openarm_mit/VlaMitController`가 `TaskSpaceImpedanceMitController`를
상속해 `write_task_target()` 하나만 override 한다 — **task/joint 모두 drive-side impedance.**

고친 기존 결함 넷: ① 청크 무검증(모르는 `rotation_type` + 빈 배열 = UB, NaN 하나로 컨트롤러
영구 먹통), ② 스트림 staleness 개념 없음(60초 goal 타임아웃까지 동작 중간 자세로 대기),
③ 그리퍼가 재생 시각이 아니라 청크 도착 시각에 발사, ⑥ 속도 피드포워드 없음.

**측정으로 갈린 결정 하나**: 코어를 `cho_controller_common`에 넣으면 안 된다. 그 패키지는
`-Ofast`로 컴파일되고 이건 `-ffinite-math-only`를 함의해 **`isfinite()`가 NaN에 true를 반환한다**
(g++ 11.4 실측). 검증기가 그 자리에서 정확히 고치려는 결함을 갖게 된다. 저장소에서 `-Ofast`를
쓰는 패키지는 그것 하나뿐이고 기존 `allFinite()` 가드들은 안전하다.

**MuJoCo 검증이 단위 테스트로 잡을 수 없는 결함 둘을 잡았다.** ① `resume_on_chunk` 기본값이
틀렸다 — hold가 `hold_timeout` → abort까지 사실상 종결 상태여서 15 Hz BEST_EFFORT 스트림에서
두 개 연속 유실이면 rollout이 끝났다. 단위 테스트는 그 latch 동작을 *설계대로* 맞다고 단정하고
있었다. 두 호스트 모두 `resume_on_stream_recovery` 기본 true로 바꿨다. ② 과거 접두사 판정이
`t <= now`라 도착시각 경로에서 매 청크의 웨이포인트 0을 버렸고(`dropped_past`가 항상 청크당 1)
지연 경보로 쓸모가 없었다 → `t < now`.

검증 도구는 `ros2 run cho_control_tools vla_mit_probe`로 저장소에 남겼다.

미해결: real bringup에서 선택 불가(의도, 이제 결정 가능한 상태) / OpenArm VLA 행동 트리 없음 /
실제 정책으로는 미검증(프로브는 합성 청크) / 브릿지 넷(openpi·GR00T·OpenVLA-OFT·LeRobot)은
저장소 밖이라 `header.stamp` 에코가 필요하고 그때까지 `chunk_time_source: arrival`이 기본이다.

상세는 §1의 2026-09-09. 설계·결정 근거는 `todo/VLA_TODO.md`,
코어의 설계 문서는 `cho_controller/utils/cho_vla_core/DESIGN.md`.

## §1 기록

### 2026-09-09

VLA 파이프라인 1~5단계를 전부 진행했다. 커밋 전 상태.

**시작은 "openarm mit에도 vla controller를 만들고 싶다"였고, 먼저 기존 Franka 코드를 평가했다.**

사용자가 "잘 짜져 있나"를 물었고, 읽어보니 제어 법칙과 RT 규율은 강했다 — `readFromRT()`를 구독
콜백에서 부르면 RealtimeBuffer의 single-RT-reader 계약을 깬다는 걸 찾아내 shadow copy로 바꾼 것,
매 사이클 재앵커링이 runaway라는 걸 sim에서 재현하고 앵커를 청크 동안 상수로 고정한 것, 포화에
`√(2·a·dist)` 제동 엔벨로프를 둔 것. 반면 **입력 검증과 스트림 생존성은 비어 있었다.**

| 결함 | 내용 |
|---|---|
| ① | `arm_actions`에 `isfinite` 검사가 한 군데도 없음. 모르는 `rotation_type` → `dim=0` → 빈 배열이 크기 검사 통과 → 역순 이터레이터 범위(UB). NaN 하나면 `q_ref_`가 영구 오염(주석이 스스로 인정) |
| ② | `goal_timeout_sec`(60초)는 **goal** 타임아웃이라 정책이 5초에 죽으면 팔이 동작 중간 자세로 55초 대기 |
| ③ | 그리퍼가 파싱 루프(non-RT)에서 발사 → "청크 안에 닫으라는 게 있으면 지금 닫는다", EE 도달보다 최대 1 추론주기 빠름 |
| ④⑤ | 재생 클럭과 상대 앵커가 도착 시각 기준 → 추론 지연만큼 항상 과거 계획을 따르고 과이동 |
| ⑥ | 위치만 출력 → velocity 모드는 컨트롤러에서 다시 미분, MIT는 `v_des`가 없어 `dq_des = J⁺v_des` 불가 |

**LeRobot을 코드로 읽은 것이 설계를 바꿨다**

사용자가 async_inference 참고를 제안해 로컬 체크아웃(`~/lerobot`)과 업스트림 `main`(2774d9bd)을
diff했다. `async_inference/`는 import 정리만 있었지만 업스트림엔 **정책 레벨 RTC**(`policies/rtc/`)와
새 롤아웃 실행기가 들어와 있었고 지원 목록에 groot가 있다 — 우리 네 스택 중 둘에 직접 걸린다.

가져온 것 중 가장 중요한 건 **"컨트롤러 클럭을 에코한다"**는 계약이다. 처음엔 "브릿지가 관측
캡처 시각을 stamp에 찍는다"고 썼는데, 그건 브릿지 wall-clock이라 이 저장소의 Multi-PC 구성에서
chrony 없이 틀어진다. LeRobot은 정수 timestep을 에코해 시각 동기를 아예 필요 없게 만든다 —
같은 원리로 **브릿지가 관측에 쓴 joint state의 stamp를 그대로 복사**하게 했다.

바꾼 것: LeRobot은 큐가 곧 클럭(틱당 1 pop)이고 신형 실행기도 **인덱스 기반** 등분 보간이다.
30 Hz 서보 버스에는 맞지만 우리는 750~1000 Hz 대 30~50 Hz 격자에 지터가 있어 시간 샘플링이
필요하다. 큐가 비면 무명령으로 두는 것도 안 된다 — 토크 제어 팔은 스스로 버티지 않고, MIT는
write 누락이 프로토콜 폴트다.

**코어를 어디에 둘지가 측정으로 뒤집혔다**

계획은 `cho_controller_common/vla`였다. 그런데 그 패키지는 `-Ofast`로 컴파일된다. `-Ofast`는
`-ffast-math` → `-ffinite-math-only`를 함의하고, g++ 11.4에서 실측하니 **NaN을 담은 벡터를
all-finite로 보고한다.** 결함 ①을 고치려는 검증기가 그 자리에서 정확히 그 결함을 갖는다.

저장소 전체를 훑어 `-Ofast`를 쓰는 패키지가 `cho_controller_common` 하나뿐이고 `allFinite()`
가드를 가진 패키지들(franka/openarm_mit/hardware)은 전부 기본 최적화라 **기존 가드는 안전함**을
확인했다. 별도 패키지 `cho_vla_core`로 옮겼고, 부수 이득으로 MIT가 eiquadprog/TSID 스택을
끌어오는 것도 피했다.

플래그 상호작용도 실측했다: `-fno-finite-math-only`는 **순서와 무관하게 `-Ofast`를 이긴다.**
그래서 tripwire 테스트가 지키는 것은 최적화 레벨이 아니라 그 플래그의 존재다 — 플래그를 지우고
`-Ofast`를 넣으면 실패하고, 플래그가 있으면 `-Ofast`를 뒤에 붙여도 통과하는 것까지 확인했다.

**MIT 이식은 새 제어법이 아니라 레퍼런스 소스 교체였다**

`TaskSpaceImpedanceMitController`를 상속하고 `write_task_target()` 하나만 override 한다. 베이스
변경은 최소(`final` 해제, `private`→`protected`, virtual화, `uses_task_space_action()` 신설).
39 인터페이스 클레임, 세션/ACK/lease/SAFE, return-to-zero 램프, drive-side 임피던스, null-space
posture, joint-limit 스프링, 마찰 FF, `max_reference_offset`이 전부 그대로 따라온다.

베이스가 허용하는 두 가지를 **설정 단계에서 거부**하게 만들었다. `max_reference_offset`은
파생 기본값(`0.5·torque_limit/kp`)을 쓰지 않고 필수로 했다 — 드라이브가 `kp(q_des−q)`를 이
컨트롤러가 클램프할 수 있는 지점 뒤에서 더하므로 이것이 임피던스 토크의 유일한 바운드이고,
운영자가 쓴 goal에 합당한 기본값이 신뢰할 수 없는 정책 출력에는 합당하지 않다. 설정값은
`min(0.25·torque_limit/kp, 0.15 rad)`: 토크 바운드만으로는 joint 2에 1.5 rad을 허용하는데 그건
임피던스 오프셋이 아니라 lunge다. `stream_timeout_sec > 0`도 필수로 했다.

**내가 틀렸다가 테스트가 정정한 것 넷**

1. `dim == 0` 가드가 죽은 코드였다. `rotation_dim()`은 잘못된 enum에 0을 주지만
   `task_waypoint_dim()`이 `3+0=3`을 반환한다. 회전 **블록** 크기를 검사하도록 고쳤다.
2. `max_task_wrench`는 drive-side에서 무효라도 **항상 필수 검증** 대상이다. 두 config 블록이
   빼먹어서 실제로 `on_configure`에 실패할 상태였고 CM 픽스처가 잡았다. 같은 종류를 앞으로
   잡도록 config 블록이 컨트롤러 요구를 만족하는지 검사하는 pytest를 추가했다.
3. `task_start_time_`을 VLA goal 시작 시 설정하지 않아 `goal_timeout_sec > 0`이면 마지막
   TaskSpace goal 시점 기준으로 측정됐다.
4. 테스트가 틀렸던 것 둘: `AngleAxisd` 추출 각도는 [0, π]로 접히는데 무한히 커지는 값과 그대로
   비교해 seqlock을 오판했고, `SE3::Interpolate`가 **나사 운동** 보간이라 회전이 동반되면
   translation이 직선 lerp가 아닌 것을 몰랐다(이건 franka 기존 동작이라 코드가 맞다).

**MuJoCo에서 돌렸고, 그것이 단위 테스트로 잡을 수 없는 결함 둘을 잡았다**

프로브(`vla_mit_probe`, 저장소에 남김)가 추론 브릿지처럼 청크를 흘려보내고 팔과 텔레메트리가
실제로 무엇을 했는지 단정한다. joint space는 q1 −0.0008 → **정확히 +0.3500**, task space는
TCP x +0.0019 → **+0.0405**(목표 +0.0419, 횡방향 1.6/3.6 mm). 최종 18/18.

첫 실행은 4개가 실패했고 원인이 하나였다. **`resume_on_chunk` 기본값이 틀렸다.** malformed 청크만
오는 구간은 워치독을 갱신하지 않으니 200 ms 뒤 hold로 갔는데, 기본 false라서 이후 정상 청크가
와도 running으로 복귀하지 못했다. 즉 hold가 abort까지 사실상 종결이고, 15 Hz BEST_EFFORT에서
두 개 연속 유실이면 rollout이 끝난다.

내 근거("정책이 왜 멈췄는지 컨트롤러는 모른다")가 틀렸다 — 실제로 죽은 경우는 `hold_timeout`이
이미 처리한다. latch는 **일시적 갭으로 goal을 죽이는 것**만 추가한다. 그리고 복귀는 계단이
아니다: hold 진입 시 참조가 released 되고 돌아온 청크는 blend로 splice 된다.

**단위 테스트가 놓친 이유가 중요하다.** `ResumeIsOptIn`이 두 분기를 다 검증하고 있었다 — latch
동작을 *설계대로* 맞다고 단정한 것이다. 테스트는 내 설계를 검증했고 설계가 틀렸다.

두 번째는 과거 접두사 판정이 `t <= now`였던 것. 도착시각 경로는 `t_obs = arrival`이라 웨이포인트
0이 정확히 `now`에 놓이고 매 청크에서 하나 버려졌다(수락 15개에 `dropped_past` 정확히 15). 지금
실행될 웨이포인트는 과거가 아니므로 `t < now`가 맞다. 고친 뒤 0.

**여전히 안 한 것.** real bringup 선택 불가(의도, 이제 결정 가능). OpenArm VLA 행동 트리 없음.
**실제 정책으로는 미검증** — 프로브는 합성 청크다. 브릿지 넷은 저장소 밖이라 `header.stamp`
에코가 필요하고 그때까지 `chunk_time_source: arrival`이 기본이다.


---

**cuRobo / cuMotion을 저장소에서 제거했다.** 전날(아래 § 2026-09-08)에 붙인 GPU 플래닝 경로를
통째로 되돌린 것이다. 판단 근거는 성능이 아니라 **의존성과 워크스페이스 설정 비용**이다:
cuRobo는 python3.10 전용 별도 venv(`~/ros2_ws/.venv-curobo`, torch cu128), 커밋할 수 없는
`COLCON_IGNORE` 마커와 그 생성 스크립트, `isaac_ros_common` shim 패키지, nvblox_msgs 스파스
체크아웃, 서브모듈 둘을 끌고 온다. 그 전부가 MoveIt 파이프라인 하나를 위한 것이었다. curobo를
쓸 일이 생기면 MoveIt 플러그인이 아니라 **외부 프로세스로 돌리고 VLA 컨트롤러 쪽으로 명령을
넣는 편이 간단하다** — 그러면 이 워크스페이스는 curobo를 전혀 알 필요가 없다.
**MoveIt은 OMPL만 쓴다.**

지운 것: 서브모듈 `extern/curobo`, `extern/isaac_ros_cumotion`(`.gitmodules`에서도 제거),
`extern/VENDORED_CUROBO.md`, `extern/nvblox_msgs_src`(+ 그 `.gitignore` 규칙),
`tools/setup_curobo_vendor.sh`, `cho_moveit/cho_moveit_curobo_deps` 패키지,
`cho_moveit_common/scripts/curobo_robot_config.py`, FR5의 `cumotion_planner.launch.py` /
`config/fr5.xrdf` / `config/isaac_ros_cumotion_planning.yaml`, `todo/CUROBO_MOVEIT_TODO.md`,
`todo/curobo_bench/`(벤치 스크립트와 CSV 전부), 그리고 `docs/installation.md`·`README.md`·
`cho_moveit/README.md`·`extern/README.md`의 해당 절.

지우기만 하면 "왜 없는지"가 사라지므로 **새 상태를 적어 뒀다. 단, 문서마다 성격에 맞는 만큼만.**
`cho_moveit/README.md`가 정본이다 — 「Planning pipeline: OMPL only」 절에 네 로봇 전부
`pipelines=['ompl']` 하나만 등록한다는 사실, 브릿지의 `planning_pipeline` 파라미터, GPU 플래너를
뺀 이유, 나중에 쓸 경우의 외부 프로세스 → `ActionChunk` 경로(미구현)를 모았다. `README.md`는 FR5
MoveIt 문단에 한 줄. `extern/README.md`에는 「Motion planners」 절로 **규칙만** — 플래너 벤더
소스를 이 폴더에 두지 않는다는 것과 설정 소유자가 `cho_moveit/`이라는 것. 사용자 지적으로 두 곳을
되돌렸다: `docs/installation.md`는 **설치할 것이 없으면 절 자체가 없어야** 하므로 넣었던 「MoveIt」
절을 뺐고, `extern/README.md`에서는 cuRobo 평가·제거 경위를 걷어냈다 — **벤더 정책 문서는 매뉴얼이지
기록이 아니다.** 경위는 이 로그가 갖는다.

고친 것: FR5 MoveIt 런치 셋(`moveit` / `move_group` / `moveit_rviz`)과
`cho_bringup_fr5/bringup_gz_moveit.launch.py`에서 `cumotion` 인자와 두 번째 파이프라인 등록을
없앴다. 액션 브릿지의 `joint_planning_pipeline` / `task_planning_pipeline` 두 파라미터는
**cuMotion이 조인트 목표를 FK로 EE 포즈로 바꿔버리는 것 때문에만 갈라 놨던 것**이라 단일
`planning_pipeline`(기본 `ompl`)로 합쳤다. `_move_goal`의 per-request 인자는 남겼다 —
파이프라인이 요청마다 실린다는 성질 자체는 MoveIt 쪽 사실이고 테스트도 그것을 검증한다.

`~/ros2_ws`에 남아 있던 잔해도 정리했다: `build/isaac_ros_cumotion_{interfaces,python_utils}`,
`install/isaac_ros_cumotion_python_utils`, 그리고 `install/cho_moveit_common/lib/`에서 끊어진
`curobo_robot_config.py` 심볼릭 링크.

검증: `cho_moveit_common` 파이테스트 21개 통과(브릿지 19 + 파라미터 2), 세 패키지 재빌드 성공,
`bringup_gz_moveit.launch.py --show-args`에 `cumotion` 없음, `MoveItConfigsBuilder`가
`pipeline_names: ['ompl']`로 확인. 아래 § 2026-09-08 기록은 당시의 측정과 판단 그대로 남긴다.


### 2026-09-08

cuRobo를 MoveIt 플래너 플러그인으로 붙였다(A안). 커밋 3개: `d28ff9d` 연동,
`3d7b2d7` 서브모듈+설치문서, `cfd1575` colcon 경계 스크립트화.

**진행 순서와 판단이 뒤집힌 지점**

처음엔 "계획 속도"가 목적이었다. Step 0(설치 없이 OMPL 기준선 측정)에서 계획 시간이 사이클의
0.1~0.2%(중앙값 5~25 ms, 궤적 5~12초)로 나와 **중단 조건에 걸렸다.** 그래서 "붙일 이유 없다"로
결론냈는데, 그건 **쉬운 씬만 본 것이었다.** 사용자가 "보통 cuRobo가 더 좋다는 인식이 크다"고
지적해 어려운 씬을 만들어 재봤고, 거기서 결론이 뒤집혔다(위 §0).

**씬 설계에 다섯 번 실패했고 그 자체가 결과다**

| 시도 | 실패 이유 |
|---|---|
| 케이지(기둥+벽+슬래브) | 통로를 안 만들고 목표를 불가능하게 만듦. OMPL 시간 불변 |
| 수직 통로 4종 | home1의 엘보가 `(-0.30,-0.15,0.45)` — 벽 놓은 자리 |
| 창 뚫린 벽 28/22/18 cm | 목표가 벽 뒤 15 cm면 손목만 아니라 엘보까지 통과해야 함 |
| 위 열린 통 30~16 cm | 목표 자세의 엘보 위치가 팔 길이(0.425 m)로 도달 불가 |
| 얇은 판(경로 측정 후 배치) | **성공** — 우회만 강제 |

핵심 원인: **home1은 팔이 작업공간 중앙에 접힌 자세**(수평 반경 0.225 m)라서 근처의 모든
장애물이 시작 자세와 충돌한다. `upright`(수평 반경 0.120 m)에서만 어려운 씬이 만들어졌다.
따라서 **이 프로젝트의 표준 시작 자세로는 OMPL이 힘들어하는 씬을 구성할 수 없다.**
어려운 씬 측정은 시작 자세가 home1이 아니므로 운용 조건 숫자와 섞어 인용하면 안 된다.

**내가 틀렸다가 측정으로 정정한 것 세 개**

1. `FINETUNE_TRAJOPT_FAIL`을 커미셔닝 가속 한계(0.7 rad/s²) 탓으로 봤다. 스윕 결과 가속
   0.7→2.0, 저크 35→500 모두 무관하고 `trajopt_tsteps` 32→64만이 원인. 계획된 궤적의 실제
   최대 가속은 0.044로 한계의 6%. 물리 한계는 애초에 구속조건이 아니었다.
2. "cuMotion이 OMPL보다 16~43배 느리다" — 틀렸다. 244 ms 중 cuRobo는 54 ms.
3. "cuRobo는 고정 GPU 작업량이라 난이도와 무관하게 상수" — 부정확. 시행당 상수이고 재시도
   횟수가 난이도에 따라 변한다(같은 씬에서 목표만 바꿔 244→83 ms).

**설계상 중요했던 발견**

cuRobo는 XRDF에서 속도 한계를 읽지 못하고 URDF 값(데이터시트 3.15 rad/s)을 쓴다. 커미셔닝
상한 1.575는 MoveIt 전용 `joint_limits.yaml`에만 있다. 그대로 두면 스케일 0.25에서
**0.7875 rad/s** — `joint_limits.yaml`이 기록한 실기 거부값(`ServoJ refused, code 14`)을
그대로 명령한다. XRDF를 지오메트리 단일 원천으로 두고 `curobo_robot_config.py`가
`velocity_scale: 0.5`를 주입하는 구조로 처리. 실측 최대 관절 속도 0.097~0.107 rad/s로 확인.

**설치에서 걸린 함정 (전부 `extern/VENDORED_CUROBO.md`에 기록)**

setuptools 59(PEP 660 없음) / 시스템 CUDA 13.2 vs torch 12.8 / `warp-lang` 상한 없어 1.17이
`wp.torch` 제거 → 1.10 고정 / `isaac_ros_common`이 안 쓰는 VPI를 무조건 요구 →
`cho_moveit_curobo_deps` shim으로 우회(상류 무패치).

**속도 스케일링 스윕** (별건, 사용자 요청): 0.25→1.0으로 4배 올려도 궤적은 2배만 줄어든다.
가속 제한 이론과 일치(1.43x vs 1.41x, 2.02x vs 2.00x). `T = A/s + B`에서 B가 1.8~2.0초로,
스케일을 무한히 올려도 그 아래로 안 내려간다. 바닥을 정하는 건 `max_acceleration: 0.7`이다.
다만 스케일 0.5는 실기가 거부한 0.7875 rad/s를 명령하므로 **실기에서 쓸 수 없는 이득**이다.

**참고만 하고 가져오지 않은 것**: `~/sdl_ws/src/sdl_project`. FR5 XRDF의 콜리전 스피어만
옮겨 적었고(이름이 정확히 일치), cuRobo는 상류에서 새로 클론했다. 그쪽 벤더본의
`motion_gen.py` 패치(hold_partial_pose 검증 우회)는 의도적으로 배제. 그 저장소는 읽기만 했고
수정 0건(mtime으로 확인).
