# Session Log

## §0 지금 상태

**AprilTag 인식이 MuJoCo에서 태스크까지 끝까지 돈다 — `tag_reach` 2회 SUCCESS.**
시뮬 로봇 + 실제 카메라 조합(MuJoCo는 이미지를 안 주므로). 검출 → `tag_9` TF → 임시 마운트
static TF → `fr3_link0` 기준 `PoseStamped` → 트리가 그 좌표로 이동 → 복귀. 게이트도 실제로
동작(손에 들면 `spread 60.3 mm` 거부, 정지하면 `0.7~3.9 mm`로 발행).

새 패키지 둘 — `cho_sensor/realsense_apriltag`(stock `rs_launch.py` include + `config_file`로 우리
yaml만), `cho_perception/cho_object_pose`(ROS 없는 `geometry.py` 코어 + tf2 어댑터). 출력이
`fr3_link0` 기준 `PoseStamped`라 `PoseTargetBehavior`가 그대로 받는다. 테스트 43개 + CI 등록.

**미해결(하드웨어): USB 3 링크가 2분을 못 넘긴다.** 첫 케이블은 협상 자체가 안 됐고, 바꾼
케이블은 협상은 되나 서로 다른 컨트롤러 두 곳에서 모두 끊긴다(8~119초). 스트리밍 없이도
끊기고 포트를 바꿔도 시점만 달라진다 → **케이블 또는 카메라 커넥터**. 다음은 세 번째 케이블.
**USB 2 폴백은 안정적**이라 480x270으로 커미셔닝은 가능하다. 살아 있던 USB 3 구간에서 848x480
실측: `fx=fy=423.94`(USB 2는 239.96) → 40 mm 태그·25 px 게이트 작업 거리 **0.68 m** vs 0.38 m.

장착(손목/삼각대) 미정으로 갈 수 있는 이유는 노드가 카메라 프레임 이름을 언급하지 않기 때문 —
`base ← tag_<id>`만 TF에 묻고 바깥에서 줄 변환은 `… → camera_link` 하나뿐이다. 상세는
`todo/APRILTAG_TODO.md`, 직전 세션은 docs/sessions/2026-09-09.md.

## §1 기록

### 2026-09-11

AprilTag 물체 위치 인식 도입. "cho_sensor에 넣으면 되지 않나"에서 시작해 설계 →
장착 미정으로 범위 조정 → Phase A 구현까지. 커밋 전 상태.

**"cho_sensor가 맞다"는 절반만 맞았다.** 파이프라인이 두 층으로 갈린다. 검출 층(카메라 +
apriltag_ros)은 `bota_ft_sensor` 패턴 그대로 cho_sensor에 들어가지만, 태그를 로봇 좌표계의
grasp pose로 바꾸는 층은 base frame 이름·grasp 오프셋·`cho_robot_config`를 알아야 해서
CLAUDE.md가 `hansung_scale`에 대해 선언한 "cho_* 와 엮지 않는다" 경계를 넘는다. →
`cho_perception/` grouping 신설.

**저장소를 읽고 확인한 것 (설계를 바꾼 것들)**

| 확인 | 영향 |
|---|---|
| `PoseTargetBehavior`는 있는데 **publish하는 노드가 repo 전체에 없다**(테스트뿐) | 목표가 "apriltag 노드를 띄운다"가 아니라 "`PoseStamped` 하나를 내보낸다"로 확정 |
| `cho_robot_config`의 `model.base_frame: world`는 **TF 트리에 없다**. 생성 URDF 루트는 `base → fr3_link0`이고 `world` 링크는 아예 없다(MoveIt launch 전용 값) | tf2 target은 `arm_base_link`. `ee_state_broadcaster.cpp:31`이 `fr3_link0`로 publish하는 것과 일치 |
| `apriltag_ros` 3.4.0 / `apriltag_msgs`가 **이미 설치돼 있다**. 미설치는 realsense2_camera·image_proc·camera_calibration | 1~3단계를 설치 없이 설계할 수 있었다 |
| `AprilTagDetection`에 **pose 필드가 없다**(hamming·decision_margin·corners·homography뿐) | 품질은 토픽에서, 자세는 TF에서 — 노드가 둘 다 써야 한다. 그리고 이게 "카메라 프레임을 모른다"를 가능하게 했다 |
| mujoco/isaac 브링업은 **사전 생성 URDF 파일**을 읽는다 | 손목 장착 시 `create_urdf.sh` 재생성 + `urdf_tests.py`가 따라온다 |

**사용자가 장착을 미정으로 돌리면서 경계를 그었다.** 그게 가능한지 따져보니 가능했다 —
`/detections`의 품질 + `lookup_transform(base, tag_<id>, 이미지 stamp)` 조합이면 카메라
프레임 이름이 노드에 등장하지 않는다. 장착에 의존하는 건 그 TF 체인의 출처 한 줄뿐이다.

**설계 판단 넷**

- **자세는 median이 아니라 medoid.** AprilTag 평면 자세의 2해 모호성 때문에 플립을 가로질러
  평균내면 *관측된 적 없는* 회전이 나오고 두 해 어느 쪽과도 가깝지 않다. 기본값은 아예 yaw만
  남기고 top-down으로 접근한다(`top_down_yaw: true`). 위치는 median.
- **게이트는 발행 *전에* 끝난다.** `PoseTargetBehavior`가 첫 메시지를 latch하므로 나쁜 포즈가
  토픽에 닿은 시점에 이미 따라간 뒤다. 점수를 붙여 내보내는 선택지가 없다.
- **`frame_prefix`** — 사용자의 "대수" 질문에서 나왔다. detector 둘을 기본값으로 띄우면 **둘 다
  `tag_9`를 publish**해서 TF 트리에 부모가 둘인 자식이 생긴다. 에러가 아니라 늦게 도착한 쪽으로
  간헐 해석되는 형태로 망가진다. detector launch와 소비자 양쪽에 prefix 파라미터를 넣었다.
- **`tag.frames`를 설정 파일에 쓰지 않는다.** launch가 `tag.ids`에서 생성하고 소비자의
  `geometry.tag_frame_name()`이 같은 규약을 쓴다 → 손으로 쓴 프레임 문자열이 양쪽에 없다.

**"카메라 위치·대수를 task로 다루면 어떤가"에 대한 결론** (상세는 `todo/APRILTAG_TODO.md` §6)

절차는 task가 맞다 — hand-eye 캘리브 task(MIT 튜닝 task가 `/mit_tuning` blackboard에 측정값을
쌓는 선례와 같은 모양), 탐색 task(`Selector(짧은 타임아웃 PoseTarget, 다음 시점)`). 반면
extrinsic은 TF에, 대수는 launch에 있어야 한다. task가 extrinsic을 들면 모든 task가 같은 숫자를
다시 적게 되고 — `cho_robot_config`가 존재하는 이유와 같은 실패 양식 — rviz/MoveIt/인식 노드는
전부 TF를 읽으므로 task가 그것들에 값을 먹일 수도 없다.

**D435 하드웨어 판단**: 컬러는 롤링 셔터, IR은 글로벌 셔터라 **infra1을 기본**으로 했다(장착이
손목이면 필수, 삼각대여도 손해 없음 — 이미 디바이스에서 rectify됨). 대가 둘: IR 도트 프로젝터를
꺼야 하고(`emitter_enabled: 0`), 태그를 **레이저 프린터로** 뽑아야 한다 — 일부 잉크젯 블랙은
근적외선 투과라 눈에는 멀쩡한 태그가 infra1에서 보이지 않는다.

**lint에서 걸린 것 (관찰)**: ament 기본 설정은 D213(요약을 둘째 줄에)을 강제하는데 이 저장소는
전부 첫 줄에 쓴다(D212 쪽, ament 규약이 무시하는 코드). 상호배타라 저장소 스타일을 택하고
`--add-ignore D213`으로 처리했다(python 패키지는 test에서, cmake 패키지는 `ament_pep257()`
인자로). 나머지 import 순서·줄 길이·인용부호는 코드를 고쳐 기본 설정으로 통과시켰다.
**별건 관찰**: 같은 명령을 `hansung_scale_driver`에 돌리면 16건이 나온다(대부분 I100).
CI TEST 목록에 있는 패키지라 확인이 필요하다 — 이번 세션에서 건드리지 않았다.

**검증**: `colcon test` 43개 통과(순수 기하 35 + cmake 패키지 lint 8). 노드 기동 확인 —
레지스트리에서 `fr3_link0` 해석, 상태 리포트 동작. mock publisher → `ros2 topic echo` 왕복 확인.
카메라 드라이버 파라미터 이름·실제 검출·launch 실행은 **미검증**(하드웨어 필요).

**같은 날 오후: apt 설치 + D435 연결 후 실기 검증.** 사용자가 "드라이버는 upstream 그대로 쓰고
파라미터만 우리 걸로 갈아끼우고 싶다"고 해서, 노드를 직접 띄우던 launch를 **`rs_launch.py`
include + `config_file`** 방식으로 바꿨다(`bota_ft_sensor`와 같은 배치). 그 과정에서 알게 된 것:

- `config_file`은 `yaml.safe_load` 결과를 그대로 넘기므로 **평탄한 매핑**이어야 한다.
  `/**: ros__parameters:` 형태로 쓰면 `'/**'`라는 파라미터 하나가 된다.
- 노드는 `[launch 인자, config 파일]` 순으로 받고 뒤가 이긴다 → `enable_color`/`enable_infra1`은
  yaml이 아니라 launch 인자여야 `stream:=`로 바꿀 수 있다.
- include를 `GroupAction(scoped=True, forwarding=False)`로 감쌌다. rs_launch가 컨텍스트의 모든
  launch configuration을 자기 목록과 대조해 경고를 찍어서, 감싸지 않으면 우리 인자 때문에
  **노란 경고 다섯 화면**이 나온다.

**추측이 틀렸던 파라미터 셋 — 전부 실측으로 교정** (이래서 설치 전 설정은 미검증으로 표시했었다)

| | |
|---|---|
| `depth_module.infra_profile` | `'WxHxF'` 형식. rs_launch 기본값이 `'0,0,0'`이라 콤마인 줄 알았는데, `'848,480,30'`은 런타임 거부 후 **조용히 기본값으로 롤백**된다 |
| `depth_module.emitter_enabled` | **이 장비엔 없다.** 드라이버가 `Projector capacity is overrided and disabled by FW`를 찍고 선언 자체를 안 한다 |
| `depth_module.exposure` | **double.** upstream이 자기 기본값 `8500`을 int로 넘겨 노드가 거부한다 — 로그의 그 경고는 우리 것이 아니다 |

**측정값** (D435 s/n 844212070094, realsense2_camera 4.58.3): `infra1/image_rect_raw` 29.99 Hz
@480x270, `/detections` 29.95 Hz(매 프레임 소화), `camera_info.frame_id =
camera_infra1_optical_frame`, 드라이버 TF `camera_link → camera_infra1_optical_frame`
RPY `[-1.571, 0, -1.571]`, intrinsics `fx=fy=239.96 cx=241.94 cy=138.22`.

**그리고 하드웨어 문제를 하나 찾았다: USB 2.** 지원 profile 목록이 424x240/480x270뿐인 게
이상해서 확인해보니 드라이버가 장비를 `RealSense USB2`(USB id `8086:0ad6`)로 보고하고
`lsusb -t`가 `480M`이다. `짧은 변 px ≈ fx · 태그크기 / 거리`이므로 실측 `fx=240`에서 40 mm 태그 +
25 px 게이트는 약 0.38 m가 한계다. USB 3이면 `fx≈424`로 약 0.68 m. §0에 액션 아이템으로 올렸다.

같은 원인으로 보이는 것 둘 더 — **노드가 `rgb_camera.*` 파라미터를 하나도 선언하지 않는다**
(color profile도 안 열린다). 그래서 `stream:=color` 경로는 노드는 정상적으로 뜨고
(`rectify_color` 실행, 토픽 생성, apriltag 구독) **데이터가 흐르지 않는다** — 검증 불가 상태다.
emitter 옵션이 없는 것도 같은 맥락. `0ad6`은 모델 PID가 아니라 USB 2 모드의 일반 식별자이고
(`lsusb`가 "RealSense 430"으로 적는 건 usb.ids의 추정), USB 3으로 바꿨는데도 color와 emitter가
안 나오면 그 모듈은 RGB가 없는 depth 전용(D430)이라는 뜻이 된다. **관찰과 추론을 구분해 둔다:
관찰은 위 네 가지, 추론은 "USB 2 모드 때문"이다.**

**케이블 교체 후 (같은 날):** `5000M`/bus 4로 올라오고 USB id가 `8086:0ad6` → `8086:0b07`,
드라이버 device name이 `RealSense USB2` → `RealSense D435`로 바뀌었다. **`0ad6`이 모델 PID가
아니라 USB 2 모드 식별자라는 추론이 확인됐다.** infra profile에 848x480(90 fps까지)·1280x800이
생기고 `rgb_camera.*` 22개가 나타났다 — USB 2가 RGB 센서를 통째로 가리고 있었던 것.
**예측이 절반 틀렸다: emitter는 USB 3에서도 없다.** `Projector capacity is overrided and
disabled by FW`가 USB 3에서도 찍히니 링크가 아니라 이 개체의 펌웨어 상태다.

**링크 안정화까지 두 단계 걸렸다.** 케이블 교체 후에도 8분간 3회 끊겼다:

    13:48:52  USB 3 up (0b07)
    13:50:26  disconnect                 <- ~94 s of streaming
    13:50:50  unable to enumerate USB device
    13:54:55  USB 3 up (0b07)            <- physical replug
    13:55:03  disconnect                 <- 8 s
    13:55:06  USB 3 up (0b07)
    13:56:34  re-enumerates on bus 3 as 0ad6, USB 3 instance disconnects

마지막이 핵심 — 장치가 사라진 게 아니라 **USB 2 버스에 `0ad6`으로 재열거**됐다. SuperSpeed만
실패하고 USB 2 폴백은 동작한 것이고, 그 전에는 폴백까지 실패했다(`error -71`,
`unable to enumerate`). 다른 컨트롤러의 포트(`2-1`)로 옮겨서 `848x480x30`을 실측할 시간은
벌었다: width 848, `fx=fy=423.94 cx=427.43 cy=245.69`, 이미지 29.1~30.0 Hz, `/detections`
**29.996 Hz** — 예측했던 `fx≈424`와 일치. **하지만 그 포트에서도 119초 만에 끊겼다.**
처음엔 "포트 교체로 해결"이라고 적었다가 곧바로 뒤집었다. 정리하면: 부하 없이도 끊기고(열거
8초 뒤 drop) 컨트롤러를 바꿔도 끊기므로 케이블 또는 카메라 커넥터가 남는다. USB 2 폴백은
여러 번 75~90초 무결함으로 안정적이다.

**프로파일을 config에서 launch 인자로 올렸다**(`profile:=`, 기본 `848x480x30`). config에 두면
launch 인자를 이길 수 없어(노드가 `[launch args, config file]` 순으로 받고 뒤가 이긴다) 링크에
따라 바꿀 수가 없다. USB 2 링크에서 848을 요구하면 드라이버가 뜨기 전에 launch가 경고한다
(실측: 경고 출력 → 드라이버가 `Setting ROS param back to: 480x270x30`). **자동 선택은 하지
않는다** — 해상도가 바뀌면 intrinsic이 바뀌므로 조용히 바꾸면 캘리브와 픽셀 임계값이 말없이
무효가 된다. `profile:=480x270x30`은 경고·에러 0, `/detections` 29.997 Hz로 확인.

USB 2에서 만들고 USB 3에서 돌리는 건 된다(사용자 계획). 넘어가지 않는 건 **intrinsic 캘리브와
거기 맞춰 튜닝한 픽셀 임계값**(`min_edge_px`) 둘뿐 — `fx` 239.96 → 423.94, 작업 거리 0.38 → 0.68 m.
코드·프레임 규약·`objects.yaml`·트리 배선·태그 크기·마운트 extrinsic은 전부 그대로 간다.

**끝까지 실증했다(오후).** 모니터에 띄운 태그로 확인 — D435의 IR 이미저는 IR-cut 필터가 없어
화면도 그냥 흑백 카메라처럼 본다. `tag36h11 id=9 hamming=0 decision_margin=206`,
TF `camera_infra1_optical_frame → tag_9 = [-0.117, 0.064, 0.349]`. 임시
`fr3_link0 → camera_link` static TF를 꽂으니 `cho_object_pose`가
`/perception/object_pose/cube`를 `fr3_link0` 기준 `[0.404, 0.091, 0.240]`으로 297건 발행했다.
**게이트 셋이 전부 현장에서 제 역할을 했다**: `edge 25.0 px < 25.0` 거부, 사용자가 카메라를
손에 들고 있는 동안 `spread 60.3 mm > 10.0` 거부, 정지하자 `spread 1.0 mm over 15 samples`로 발행.
남은 건 intrinsic 캘리브와 계측 정확도(config 태그 크기가 실제 띄운 것과 맞아야 한다).

**rviz 경로를 패키지에 넣었다**(`rviz:=true` → `apriltag_draw` + `rviz/apriltag.rviz`).
함정 둘: `apriltag_draw`는 **lazy 구독**이라 `/image_tags`를 누가 구독해야 돌고, 검출 입력
토픽이 **`tags`**지 `detections`가 아니다. 후자로 remap하면 조용히 아무 일도 안 하고 증상이
"태그 검출 안 됨"과 구별되지 않는다 — 실제로 여기서 한 번 속았다.

**MuJoCo 통합 테스트: `tag_reach` 2회 SUCCESS.** 시뮬 로봇 + 실제 카메라. 트리 로그가
`[Detect_Tag] target = [+0.37293, +0.03339, +0.16008] m in 'fr3_link0'` →
`[Move_To_Tag_Standoff] Action Succeeded!` → `Status.SUCCESS`까지 남긴다. 도달 판정은 액션
서버의 수렴이고, **TCP 독립 측정은 아직 없다**(재보려던 회차에 카메라가 죽었다).

**테스트가 버그를 하나 잡았다**: `tag_reach`에 컨트롤러 전환이 빠져 있었다. `home_subtree`가
joint impedance를 켜둔 채 끝나는데 task-space 액션 서버는 자기 컨트롤러가 active여야 답한다.
`ur/multi_move.py` 패턴대로 검출과 이동 **사이에** 스위치를 넣고(대기 중 팔은 계속 잡힘),
컨트롤러 이름도 `robot_config['task_space']`에서 가져오게 했다. 순서는 테스트로 고정.

**시간 기준 함정**: MuJoCo 기본 `use_sim_time:=true`면 로봇 TF가 sim clock인데 실제 카메라는
wall clock이라 이미지 stamp 조회가 extrapolation 에러가 된다. 하이브리드는
**`use_sim_time:=false`로 시계를 하나로**.

**safe-abort 검증**: 카메라가 죽은 회차에서 검출 15초 타임아웃 → hold 컨트롤러 전환 →
`Status.FAILURE`. 그냥 죽지 않는다.

**upstream 불안정 둘**: `realsense2_camera_node`가 **실행 중에도** segfault(15:23:58), USB 링크는
여전히 몇 분 단위로 끊긴다 — 5회 중 2회 실패가 이것 때문이다. 태스크 결함이 아니다.

**태스크 통합: 일반은 `cho_object_pose`, 구체는 태스크.** 사용자 제안이고 동의했다. 파이프라인·
게이트·프레임 해석은 perception이 갖고, "어느 태그가 무슨 물체이고 어디로 가나"는 태스크가
갖는다. 갈라 놓은 기준 — 태스크: 물체 테이블 + `min_samples`/`max_position_spread_m`(요구
정확도) / perception: `max_hamming`·`min_decision_margin`·`min_edge_px`(광학의 함수) /
어느 쪽도 아님: `base_frame`(`cho_robot_config`에서 읽어 드리프트 차단).

`run_task_manager.launch.py`에 `object_pose_config:=`를 넣었고 **비어 있으면 include 자체를
안 한다**(실측: `task:=pick_place`에서 `object_pose_node` 0개). 의존은 exec_depend 하나, import
없음. 첫 소비자는 `tasks/franka/tag_reach.py` — home → `PoseTargetBehavior` → 
`TaskSpaceActionBehavior(target_pose_key=...)` → home, **트리에 좌표가 한 줄도 없다.** 테이블은
태그 면 100 mm standoff로 잡았다(캘리브 전이라 스케일 오차를 충돌 아닌 빗나감으로).

부수 수정 하나: compat view `controller_names.load_robot_config`에 `arm_base_link`를 추가했다.
거기엔 `model`이 없어 `KeyError`가 났는데, 태스크마다 레지스트리를 다시 열거나 프레임을 문자열로
박는 것보다 낫다.

**린트 진단을 한 번 틀렸다.** `cho_task_manager`에 pep257 위반 102건이 보이길래 "99건이 기존
문제"라고 했는데, 그 패키지의 `test_pep257.py`는 **명시적 `--ignore` 목록**(D213 포함)을 넘긴다.
CLI 기본 인자로 돌린 내 관찰이 테스트와 달랐던 것이고, 실제로는 통과 중이었으며 내가 넣은 D301
하나가 깨뜨린 것이었다. 린터는 **패키지 테스트가 부르는 방식 그대로** 돌려야 한다.
(hansung_scale_driver 건은 실제 테스트를 돌려서 확인한 것이라 그대로 유효하다.)

**장치 선택은 포트와 무관하다**(사용자 질문). 드라이버는 처음 발견한 RealSense를 잡고, 두 대
이상이면 `serial_no`(기본 선택지) / `usb_port_id` / `device_type` 중 하나를 config에 넣는다.
시리얼은 librealsense가 보고하는 값(`844212070094`)이고 커널 USB 디스크립터의
`SerialNumber`(846623021037)와는 다른 필드다. 파이썬 바인딩(`pyrealsense2`)은 설치돼 있지
않다 — `ros-humble-librealsense2`는 C++ 라이브러리와 `rs-*` CLI만 준다.
손목 장착은 이 문제를 악화시키므로(§4④⑧) 브래킷 전에 케이블 경로를 정리할 것.

중간에 시리얼이 바뀐 줄 알고 "다른 개체"로 의심했는데 아니었다 — 커널 USB 디스크립터의
`SerialNumber`(846623021037)와 librealsense가 보고하는 장치 시리얼(`844212070094`)은 서로 다른
필드다. 같은 카메라다.

부수 관찰: `apriltag_node`와 `realsense2_camera_node`가 launch 종료 때마다 libc 안에서
segfault 한다(`journalctl -k`). 종료 경로이고 둘 다 upstream 코드다.

**케이블이냐 포트냐는 sysfs가 답한다.** xHCI는 USB2 버스와 SuperSpeed 버스를 따로 노출하고,
물리 커넥터가 USB 3이면 두 포트 객체가 `peer` 심볼릭 링크로 연결된다:

    /sys/bus/usb/devices/usb3/3-0:1.0/usb3-port4/peer -> ../../../usb4/4-0:1.0/usb4-port4

카메라가 꽂힌 4번 포트에 짝이 **있다** → 포트는 USB 3 지원. 그런데도 480M으로 올라왔으니
SuperSpeed 링크가 안 선 것이고, 남는 원인은 **케이블**(또는 접촉)이다. 카메라는 허브를 거치지
않고 루트 허브 직결이라 중간 변수도 없다. 이 컨트롤러에서 짝이 있는 포트는 2·3·4·9·10번뿐이고
나머지(1, 5~8, 11~14)는 USB 2 전용이다.

디버깅 중 헛발질 하나: `pkill -f "apriltag.launch.py"`가 **자기 자신의 셸 명령줄까지 매칭해서**
launch가 시작도 못 하고 죽었다. 같은 명령 안에서 pkill과 launch를 같이 쓰면 안 된다.

**설계 결정 하나 뒤집혔다**: `publish_tf`를 처음에 `false`로 뒀는데 틀렸다. 그건 카메라 **내부**
체인(`camera_link` → optical frames)이고 apriltag가 태그를 optical frame에 매단다 — 끄면 태그가
로봇까지 갈 길이 없다. `true`로 바꾸고, 바깥에서 줄 변환은 `… → camera_link` **하나뿐**이라고
문서에 못박았다. `realsense2_description`으로 optical frame까지 URDF에 넣으면 같은 static
변환의 퍼블리셔가 둘이 된다.

아직 못 한 것: 인쇄된 태그가 없어 **실제 검출은 확인 못 했다**. intrinsic 캘리브도 미실시.

**Franka `joint_trajectory_controller` 제거.** 이건 이름과 달리 stock JTC가 아니라 joint별
harmonic(sin/cos) 여기 궤적을 돌리며 로그를 남기는 **일회성 system-identification 컨트롤러**였다.
어떤 작업 중에 만들어진 것이고 프로젝트 범위에 안 맞는다는 사용자 판단으로 삭제.

지운 곳: 소스/헤더 2개, `CMakeLists.txt`, `cho_controller_franka.xml` 플러그인 선언,
real·gazebo `controllers.yaml`의 인스턴스 + 파라미터 블록, 두 launch의
`extra_torque_controllers`, `docs/controllers_and_bringup.md` 표(14 → 13개),
CLAUDE.md 컨트롤러 목록, `todo/CONTROLLER_STABILITY_TODO.md`의 kd 언더댐핑 항목(함께 무의미해짐).

**남긴 것과 그 이유**: 이름이 비슷한 `moveit_joint_trajectory_controller`는 upstream
`joint_trajectory_controller/JointTrajectoryController`로, MoveIt 실행 백엔드라 별개다.
gazebo config에 둘을 구분하려고 달아둔 주석("위쪽 Cho 컨트롤러는 별도의 identification 궤적")도
이제 가리킬 대상이 없어 정리했다. `cho_bringup_franka/package.xml`의 `joint_trajectory_controller`
exec_depend도 이쪽 용도라 유지. UR/FR5/OpenArm의 동명 컨트롤러는 전부 stock JTC로 무관.

`cbp cho_controller_franka` 빌드 통과(기존 sign-compare 경고만).

- 2026-09-09 VLA 코어 분리 + OpenArm 컨트롤러 통합 + cuRobo 경로 제거 → docs/sessions/2026-09-09.md
- 2026-09-08 cuRobo를 MoveIt 플래너 플러그인으로 연동(09-09에 제거) → docs/sessions/2026-09-08.md
