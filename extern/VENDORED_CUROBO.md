# 벤더링 기록 — cuRobo / isaac_ros_cumotion

`todo/CUROBO_MOVEIT_TODO.md` A안(cuMotion MoveIt 플래너 플러그인)을 위해 받은 소스.
2026-09-08 취득.

## 무엇을 받았나

| 경로 | 상류 | 리비전 | 라이선스 |
|---|---|---|---|
| `extern/curobo` | [NVlabs/curobo](https://github.com/NVlabs/curobo) | 태그 `v0.7.8` = `d64c4b005459db10c5dd867d8b30a87d5bda9bdb` (검증됨) | **NVIDIA License — §3.3 non-commercial** |
| `extern/isaac_ros_cumotion` | [NVIDIA-ISAAC-ROS/isaac_ros_cumotion](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion) | `release-3.2`, `dbaa7e8` (shallow) | 패키지별 Apache-2.0 / 저장소 루트는 NVIDIA Isaac ROS Software License |
| `extern/isaac_ros_common` | [NVIDIA-ISAAC-ROS/isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) | `release-3.2`, `fcf4d9e` (shallow) | Apache-2.0 |
| `extern/nvblox_msgs_src/nvblox_msgs` | [NVIDIA-ISAAC-ROS/isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) | `release-3.2` (sparse checkout, 이 패키지만) | Apache-2.0 |

아직 **git submodule로 등록하지 않았다.** 기존 `extern/`은 서브모듈 방식이고 `.gitmodules`는
추적 파일이므로, 등록은 별도 결정으로 남긴다. 지금은 untracked 체크아웃이다.

## 라이선스 — 진행 전제

`extern/curobo`(v0.7.x)는 NVIDIA License이고 §3.3에 다음이 있다:

> The Work and any derivative works thereof only may be used or intended for use
> non-commercially. ... "non-commercially" means for research or evaluation purposes only.

Apache-2.0인 것은 cuRobo **v0.8.0(cuRoboV2)**와 `isaac_ros_cumotion*` 래퍼뿐이다.
Humble용 `isaac_ros_cumotion`(release-3.2)의 플래너 노드가 v1 API
(`curobo.wrap.reacher.motion_gen.MotionGen`)를 import하므로 v1에 묶이고, 이 제한을 상속한다.
**연구/평가 목적 전제로 진행한다.** 상용화·기술이전이 논의되면 재판단이 필요하다.

## 상류 무패치

상류를 수정하지 않는다. 특히 `~/sdl_ws`의 cuRobo 벤더본에 있는 `motion_gen.py` 패치
(`update_pose_cost_metric()`이 hold_partial_pose 검증을 통과시키도록 `True` 반환)는
**가져오지 않았다.** 그 프로젝트의 `VENDORED.md`도 "그 브랜치에 도달하는 모든 caller에
적용되며 transfer 태스크에 한정되지 않는다"고 스스로 경고한다. 그쪽엔 사후 fail-closed
가드가 있지만 이 프로젝트엔 없다.

## colcon 경계 — `extern/README.md` 정책과의 차이

`extern/README.md`는 "패키지 이름이 `cho_*`와 충돌하지 않으므로 upstream 체크아웃에
`COLCON_IGNORE`를 넣는 대신 명시적 allowlist로 경계를 강제한다"고 적고 있다.
여기서는 **발견 단계에서 막았다.** 이유는 두 가지다:

1. `curobo_core`는 빌드 시점에 시스템 python의 torch를 요구하고 `curobo/` 서브모듈이
   비어 있어, 통상 빌드(`cbr`)에서 **실패한다.** 실패하는 패키지를 워크스페이스에 남길 수 없다.
2. 두 저장소를 그대로 두면 쓰지 않는 패키지 19개가 매 전체 빌드에 추가된다.
   이 머신은 빌드 워커가 2개로 제한돼 있어(`CLAUDE.md`) 실질 비용이다.

그래서 allowlist 6개(`isaac_ros_common`, `isaac_ros_cumotion`, `_interfaces`,
`_python_utils`, `_robot_description`, `_moveit`, 그리고 `nvblox_msgs`)만 남기고 나머지
19개에 `COLCON_IGNORE`를 넣었다. 각 파일에 이유가 적혀 있고, 지우면 되살아난다.
`extern/curobo`에도 넣었다 — ROS 패키지가 아니라 pip 소스 트리인데 colcon이 `setup.py`를
보고 python 패키지로 오인하기 때문이다.

## 파이썬 환경

`curobo_core`(colcon 패키지) 경로를 **쓰지 않는다.** 그것은 시스템 python에 torch가 있는
Isaac ROS 도커를 전제한다. 대신 `todo/CUROBO_MOVEIT_TODO.md` D3대로:

```
~/ros2_ws/.venv-curobo    python3.10 (시스템과 동일 3.10.17), torch 2.7.0+cu128
```

- **3.10은 강제다.** Humble rclpy는 `_rclpy_pybind11.cpython-310-*.so`이므로 3.11/3.12는 불가.
- rclpy는 venv 안이 아니라 `/opt/ros/humble` PYTHONPATH에서 온다. 실측 확인됨.
- venv에 numpy가 있어야 `sensor_msgs`가 import된다 (cuRobo가 요구하므로 자연히 들어온다).
- cuRobo 커널은 torch에서 아키텍처를 상속하므로 이 GPU(sm_120)로 컴파일된다.

## 설치 재현 (2026-09-08 실측, 함정 4개 포함)

```bash
python3.10 -m venv ~/ros2_ws/.venv-curobo
~/ros2_ws/.venv-curobo/bin/pip install -U pip wheel "setuptools>=70,<81" setuptools_scm
~/ros2_ws/.venv-curobo/bin/pip install torch==2.7.0 --index-url https://download.pytorch.org/whl/cu128

cd extern/curobo
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST="12.0"
export MAX_JOBS=4
~/ros2_ws/.venv-curobo/bin/pip install -e . --no-build-isolation

# cuRobo 0.7.8 이 상한을 걸지 않은 의존성 두 개를 되돌린다 (아래 함정 3)
~/ros2_ws/.venv-curobo/bin/pip install "warp-lang==1.10.0" "trimesh==4.9.0"
```

### 함정 1 — venv의 setuptools가 PEP 660을 모른다

`python3.10 -m venv`는 우분투 22.04의 **setuptools 59.6.0**을 심는다. 편집 설치(`-e`)는
PEP 660 `build_editable` 훅을 요구하고 그것은 setuptools 64+에만 있다. 증상:

```
ERROR: Project ... uses a build backend that is missing the 'build_editable' hook
```

→ venv 생성 직후 setuptools를 올린다.

### 함정 2 — 시스템 기본 CUDA가 torch와 다르다

이 머신의 `/usr/local/cuda`는 **13.2**를 가리키고 torch는 **12.8**로 빌드돼 있다. 증상:

```
RuntimeError: The detected CUDA version (13.2) mismatches the version that was
used to compile PyTorch (12.8).
```

→ `CUDA_HOME=/usr/local/cuda-12.8`을 명시한다. 이 머신에는 12.8 툴킷이 설치돼 있다.

### 함정 3 — cuRobo 0.7.8의 의존성 상한이 없다

`setup.cfg`가 `warp-lang>=0.9.0`, `trimesh`(무제한)로 선언하므로 pip가 최신을 가져온다.
`warp-lang 1.17.0`에서는 `wp.torch` 접근자가 사라져 즉시 깨진다:

```
AttributeError: module 'warp' has no attribute 'torch'
```

→ `~/sdl_ws`에서 실제로 동작하는 조합인 **warp-lang 1.10.0 / trimesh 4.9.0**으로 고정한다.
(그 환경의 나머지: torch 2.7.0+cu128, numpy 2.2.6, yourdfpy 0.0.58 — yourdfpy는 0.0.60도 무해했다.)

### 함정 4 — `TORCH_CUDA_ARCH_LIST`를 명시하지 않으면 자동탐지에 의존한다

기본값은 현재 GPU를 보고 정하므로 이 머신에서는 결과가 같지만, 다른 GPU에서 빌드하거나
GPU가 안 보이는 환경(도커 빌드)에서는 달라진다. `12.0`을 명시했다.

### 설치 검증

```bash
# 커널이 sm_120 으로 컴파일됐는지
for so in extern/curobo/src/curobo/curobolib/*.so; do
  /usr/local/cuda-12.8/bin/cuobjdump -lelf "$so" | grep -oE "sm_[0-9]+" | sort -u
done
```

2026-09-08 결과: 5개 커널(`geom_cu`, `kinematics_fused_cu`, `lbfgs_step_cu`,
`line_search_cu`, `tensor_step_cu`) 전부 `sm_120`.

FR5 설정으로 실제 계획까지 확인했다 (`MotionGen` 생성 5.1 s + warmup 4.5 s는 런치 시 1회):

| 항목 | 값 |
|---|---|
| solve 시간 | 63~66 ms |
| 궤적 길이 | 6.6~6.8 s |
| 최대 관절 속도 | 0.097~0.107 rad/s |

마지막 줄이 `velocity_scale: 0.5`가 실제로 적용됐다는 증거다 — 커미셔닝 상한
`1.575 x time_dilation 0.25 = 0.394 rad/s` 아래에 들어온다.

## 미해결 — `COLCON_IGNORE` 가 추적되지 않는다

위 §colcon 경계에서 벤더 체크아웃 **안에** `COLCON_IGNORE` 24개를 넣었다. 그 파일들은 각
서브레포 안에 있고 이 저장소는 벤더 트리를 커밋하지 않으므로(`.gitignore`), **추적되지 않는다.**

결과: 이 저장소를 새로 클론하고 위 레시피대로 벤더 소스만 받은 사람은 `COLCON_IGNORE` 가
없는 상태가 되고, `cbr`(전체 빌드)이 `curobo_core` 에서 실패한다 — 시스템 python 에 torch 가
없고 `curobo/` 서브모듈이 비어 있기 때문이다.

**설계 실수다.** 경계는 이 저장소가 소유하는 방식이어야 한다. 후속으로 둘 중 하나:

1. `tools/build_curobo_vendor.sh` — OpenArm 의 `tools/build_openarm_vendor.sh` 와 같은
   allowlist 빌드 스크립트. 벤더 트리를 수정하지 않고 `--packages-select` 로 경계를 강제한다.
   `extern/README.md` 정책과도 맞는다.
2. 위 레시피에 `COLCON_IGNORE` 생성 단계를 명시해 사람이 실행하게 한다. 싸지만 잊기 쉽다.

1번이 맞다. 그때 이 절과 §colcon 경계를 함께 정리한다.
