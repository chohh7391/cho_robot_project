# hansung_scale_driver

한성전자저울(Hansung/한성계기) **HS-AA 시리즈** 전자저울을 RS232로 읽는 ROS2(Humble) 드라이버입니다.

**읽기 전용입니다.** 이 저울의 RS232 포트는 단방향 출력 전용이라는 것을 제조사에서 확인받았고, 실기 테스트로도 확인했습니다([아래](#rs232-단방향-확인-내역)). 영점·용기값(tare)은 저울 물리 버튼으로만 가능하고, 드라이버는 저울이 뱉는 값을 읽어서 발행하는 일만 합니다.

프로토콜 문서가 공개되어 있지 않아 실제 장비를 시리얼 스니핑해서 역으로 확인했습니다. 아래는 전부 **실측값**입니다.

패키지 구조와 인터페이스는 [realsense-ros](https://github.com/realsenseai/realsense-ros)의 `realsense2_camera` 래퍼를 참고했습니다.

## 확인된 프로토콜

| 항목 | 값 |
|---|---|
| Baud rate | **2400 bps** |
| 데이터 / 패리티 / 정지 비트 | **8 / None / 1** |
| 동작 방식 | 연속 출력 (요청 없이 계속 스트리밍) |
| 프레임 주기 | **5.000 Hz** (간격 192~209 ms) |
| 통신 방향 | **단방향 (저울 → PC)** |

프레임 형식:

```
WT<status:2><sign:1>   <value>   <unit>\r\n

예: WTST+  12.70   g\r\n
```

| 필드 | 설명 |
|---|---|
| `WT` | 고정 헤더 |
| `status` | `ST` = 안정(Stable). 실기에서 확인된 값은 `ST`뿐입니다 |
| `sign` | `+` 또는 `-` |
| `value` | 공백 좌측 패딩, 소수점 2자리 |
| `unit` | 공백 좌측 패딩 (`g` 확인) |

`US`(불안정)·`OL`(과부하)는 이 프레임 형식의 관례를 따른 **추정치**이고 실기 확인 전입니다. 그래서 `stable`은 `status != "US"`가 아니라 **`status == "ST"`일 때만** true입니다 — 모르는 코드를 안정으로 간주하면 안 되니까요.

---

# 빠른 시작

## 1. 시리얼 포트 권한

USB-RS232 어댑터는 `dialout` 그룹만 접근할 수 있습니다. **최초 1회**만 하면 됩니다.

```bash
sudo usermod -aG dialout $USER
```

적용하려면 **로그아웃 후 재로그인**(또는 새 터미널에서 `newgrp dialout`)이 필요합니다. 지금 당장 한 번만 써야 한다면 임시로:

```bash
sudo chmod a+rw /dev/ttyUSB0     # 어댑터를 다시 꽂으면 초기화됨
```

권한이 없으면 노드가 이렇게 알려줍니다:

```
[ERROR] Failed to open /dev/ttyUSB0: [Errno 13] Permission denied
        The user is not in the dialout group: `sudo usermod -aG dialout $USER`,
        then log out and back in.
```

## 2. 빌드

```bash
cd ~/rs232_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hansung_scale_msgs hansung_scale_driver --symlink-install
source install/setup.bash
```

`hansung_scale_msgs`가 먼저 빌드됩니다. `ament_python` 패키지는 ROS 인터페이스를 생성할 수 없어서, realsense-ros가 `realsense2_camera_msgs`를 분리한 것과 같은 이유로 메시지를 별도 `ament_cmake` 패키지로 뺐습니다.

## 3. 어댑터 확인

어떤 포트에 뭐가 붙어 있는지, 그리고 **그대로 복붙할 수 있는 선택자**를 출력합니다.

```bash
ros2 run hansung_scale_driver scale_sniffer --ros-args -p list_ports:=true
```

```
DEVICE               VID:PID    SERIAL             USB PORT     DESCRIPTION
---------------------------------------------------------------------------
/dev/ttyUSB0         0403:6001  FTEFY2BT           8-1          USB Serial Converter
/dev/ttyS0           -          -                  -            n/a
... (커널이 무조건 만드는 레거시 포트들)

# Stable selectors for scale_node (any one of these):
#   -p serial_no:=FTEFY2BT      (/dev/ttyUSB0)
#   -p usb_port_id:=8-1         (/dev/ttyUSB0)
#   -p port:=/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTEFY2BT-if00-port0
```

USB 어댑터를 위로 정렬해서 보여줍니다. `/dev/ttyUSB0` 같은 번호는 USB 열거 순서가 바뀌면 같이 바뀌니, **`serial_no`를 쓰는 걸 권장합니다.**

## 4. 실행

```bash
ros2 launch hansung_scale_driver scale.launch.py serial_no:=FTEFY2BT
```

```
[INFO] Read-only mode: ~/cmd, ~/tare, ~/zero and ~/send_command are not advertised.
[INFO] Configured /dev/ttyUSB0 0403:6001 sn=FTEFY2BT usb_port_id=8-1 @ 2400 8N1
[INFO] Activated: streaming weight data
```

## 5. 값 확인

```bash
ros2 topic echo /scale_node/weight_stamped
```

```yaml
header:
  stamp: {sec: 1788770099, nanosec: 279449507}
  frame_id: scale_link
weight: 12.7          # 저울이 말한 값 그대로
unit: g
weight_grams: 12.7    # 그램 환산 (모르는 단위면 NaN)
stable: true          # status == ST 일 때만 true
status: ST
```

---

# 인터페이스

## 발행 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `~/weight_stamped` | `hansung_scale_msgs/WeightStamped` | **권장.** 타임스탬프 + 무게 + 단위 + 그램 환산 + 안정 여부 + status를 한 메시지로 |
| `~/weight` | `std_msgs/Float32` | 부호 반영된 무게 값 |
| `~/stable` | `std_msgs/Bool` | 안정 여부 |
| `~/unit` | `std_msgs/String` | 단위 문자열 |
| `~/raw` | `std_msgs/String` | 파싱 전 원본 라인 (디버깅용) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `diagnostics_period > 0`일 때 |

`~/weight`/`~/stable`/`~/unit`은 세 개의 별도 메시지라 소비자가 시간 상관관계를 직접 맞춰야 합니다. **새로 쓰는 코드는 `~/weight_stamped`를 쓰세요.**

`~/stable`과 `~/unit`은 **완전한 HS-AA 프레임을 파싱했을 때만** 발행됩니다. `value_regex` 보조 파서로 숫자만 뽑아낸 라인에서 기본값(불안정·단위없음)을 흘리면 진짜 측정값과 구분이 안 되기 때문입니다.

## 서비스

| 서비스 | 타입 | 설명 |
|---|---|---|
| `~/device_info` | `hansung_scale_msgs/DeviceInfo` | 실제로 열린 포트, 어댑터 시리얼번호, 라인 설정, 프레임 통계 |
| `~/hw_reset` | `std_srvs/Trigger` | 포트를 닫고 다시 열면서 DTR 펄스 (어댑터가 먹었을 때) |

`~/cmd`, `~/tare`, `~/zero`, `~/send_command`는 **기본적으로 존재하지 않습니다.** 이 저울이 입력을 받지 못하므로, 동작할 수 없는 서비스를 `ros2 service list`에 띄워두지 않습니다. 입력을 받는 다른 인디케이터를 붙이면 `enable_commands:=true`로 살아납니다.

## 파라미터

`config/scale_params.yaml`에 전부 주석과 함께 있습니다. `ros2 param describe /scale_node <이름>`으로도 설명을 볼 수 있습니다.

### 장치 선택

`serial_no`/`usb_port_id`/`device_type` 중 하나라도 설정되면 `port`는 **무시**됩니다. 기본값으로 남아 있는 `port`가 명시적으로 설정한 선택자를 이기지 않도록 한 것입니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 장치 경로 또는 `/dev/serial/by-id/...` 심볼릭 링크 |
| `serial_no` | `''` | **권장.** 어댑터 USB 시리얼번호. 재부팅·재연결에도 안 바뀜 |
| `usb_port_id` | `''` | 물리 USB 위치 접두사 (예: `8-1`). 시리얼번호 없는 어댑터용 |
| `device_type` | `''` | 어댑터 description/manufacturer/product 매칭 정규식 (예: `ftdi`) |
| `wait_for_device_timeout` | `-1.0` | configure 시 포트가 나타날 때까지 기다리는 초. 음수/0이면 한 번만 시도 |
| `reconnect_timeout` | `6.0` | 링크가 끊긴 뒤 재시도할 초. 음수면 첫 읽기 에러에서 리더 정지 |
| `initial_reset` | `false` | 연결 직후 DTR 펄스 + 입력 버퍼 flush |

### 라인 설정

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `baudrate` | `2400` | 실측값 |
| `bytesize` | `8` | 5, 6, 7, 8 |
| `parity` | `'NONE'` | `NONE`/`EVEN`/`ODD`/`MARK`/`SPACE` |
| `stopbits` | `1.0` | 1, 1.5, 2 |
| `timeout` | `1.0` | pyserial 읽기 타임아웃(초) |

> `parity`를 한 글자 `'N'`이 아니라 `'NONE'`으로 쓰는 이유: YAML 1.1에서 맨 `N`/`Y`는 **boolean 리터럴**입니다. params 파일이나 launch 치환을 거치는 순간 `False`로 조용히 변합니다. (한 글자 표기도 여전히 받아주긴 합니다.)

### 프로토콜

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `line_ending` | `'\r\n'` | 프레임 구분자. 이스케이프 또는 `hex:0D0A` |
| `value_regex` | `'[-+]?\d+\.?\d*'` | `WT...` 프레임에 안 걸리는 라인용 보조 파서 |
| `frame_id` | `'scale_link'` | `~/weight_stamped`의 frame_id |

### 토픽 / QoS

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `enable_weight_stamped`, `enable_weight`, `enable_stable`, `enable_unit`, `enable_raw` | `true` | 개별 퍼블리셔 생성 여부 |
| `weight_qos` | `'SYSTEM_DEFAULT'` | 무게 토픽 QoS 프리셋 |
| `raw_qos` | `'SYSTEM_DEFAULT'` | `~/raw` QoS 프리셋 |

프리셋: `SYSTEM_DEFAULT`, `DEFAULT`, `SENSOR_DATA`, `SERVICES_DEFAULT`, `PARAMETERS`, `PARAMETER_EVENTS`. 최신값만 필요한 소비자라면 `SENSOR_DATA`(best-effort, 작은 큐)가 맞습니다. 오타는 조용히 기본값으로 떨어지지 않고 configure 실패로 잡힙니다.

### 진단

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `diagnostics_period` | `1.0` | `/diagnostics` 발행 주기(초). 0이면 비활성 |
| `expected_frame_rate` | `5.0` | 기대 프레임 주기. **실측 5.000 Hz.** staleness 판정 기준 |

### 기타

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `enable_commands` | `false` | 쓰기 인터페이스 노출 여부. 이 저울은 입력을 못 받음 |
| `poll_mode` / `poll_command` / `poll_interval` | `false` / `''` / `0.5` | 요청/응답형 인디케이터용. 이 저울에는 해당 없음 |
| `tare_command` / `zero_command` / `command_response_timeout` | `''` / `''` / `0.0` | `enable_commands`가 true일 때만 의미 있음 |
| `autostart` | `true` | 시작 시 자동 configure + activate |

### 런타임 변경 가능한 파라미터

`value_regex`, `frame_id`, `poll_interval`, `tare_command`, `zero_command`, `command_response_timeout`, `expected_frame_rate`.

나머지는 configure 시점에만 읽으므로, configure된 노드에 설정하려 하면 **거부하면서** 어떻게 해야 하는지 알려줍니다. 조용히 받아놓고 반영하지 않는 것보다 낫습니다.

```bash
ros2 param set /scale_node frame_id weigh_cell   # → Set parameter successful
ros2 param set /scale_node baudrate 9600
# → Setting parameter failed: baudrate is only read while configuring.
#    Run `ros2 lifecycle set scale_node cleanup` then `configure` to apply it.
```

---

# 자주 하는 일

## 안정된 값이 나올 때까지 기다렸다 읽기

로봇 시퀀스에서 쓰는 기본 패턴입니다. `stable`이 true인 프레임만 받아서 씁니다.

```python
import rclpy
from rclpy.node import Node
from hansung_scale_msgs.msg import WeightStamped


class WeighOnce(Node):
    """Wait for a settled reading, print it, shut down."""

    def __init__(self):
        super().__init__('weigh_once')
        self.create_subscription(WeightStamped, '/scale_node/weight_stamped',
                                 self.on_weight, 10)

    def on_weight(self, msg):
        if not msg.stable:
            return                      # 아직 흔들리는 중
        self.get_logger().info(f'{msg.weight} {msg.unit} ({msg.weight_grams} g)')
        raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(WeighOnce())
    except SystemExit:
        pass
    rclpy.shutdown()
```

`stable`이 계속 false라면 저울이 아직 안정 판정을 안 내린 상태입니다. 프레임 자체는 5 Hz로 계속 오니 `~/raw`로 실제 status 값을 확인해보세요.

## 네임스페이스 / 노드 이름 바꾸기

여러 대를 붙이거나 셀 단위로 묶을 때 씁니다.

```bash
ros2 launch hansung_scale_driver scale.launch.py serial_no:=FTEFY2BT \
  scale_namespace:=/cell1 scale_name:=weigh_station
# → /cell1/weigh_station/weight_stamped
```

`config/scale_params.yaml`은 `/**` 와일드카드로 키가 잡혀 있어서 이름을 바꿔도 그대로 적용됩니다.

## 파라미터 파일 따로 쓰기

```bash
ros2 launch hansung_scale_driver scale.launch.py params_file:=/path/to/my_scale.yaml
```

launch 인자가 **기본값 그대로면 노드에 전달하지 않습니다.** 그래야 `params_file`의 값이 실제로 적용됩니다. (`rs_launch.py`는 전부 무조건 전달해서 params 파일이 무시된 것처럼 보이는데, 그 동작만 의도적으로 다르게 했습니다.)

## lifecycle 제어

`unconfigured → inactive → active` 상태를 갖는 관리형 노드입니다. 포트는 **configure 시점에만** 열리고, **active 상태에서만** 발행합니다.

```bash
ros2 lifecycle get /scale_node
ros2 lifecycle set /scale_node deactivate   # 포트는 유지, 발행만 중단
ros2 lifecycle set /scale_node activate
```

외부 lifecycle 매니저에 맡기려면 `autostart:=false`로 띄우면 됩니다.

## 상태 감시

```bash
ros2 topic echo /diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor
```

```yaml
name: 'scale_node: RS232 link'
message: Streaming at 5.0 frames/s
values:
  frame_rate_hz: '5.00'          expected_frame_rate_hz: '5.00'
  frames_received: '1284'        parse_errors: '0'
  dropped_bytes: '0'             last_frame_age_s: '0.14'
  last_weight: 12.7 g            last_status: ST            stable: 'True'
```

링크 다운(ERROR), 프레임 끊김(WARN), 파싱 실패(WARN)를 구분해서 알려줍니다. lifecycle publisher가 아니라 일반 publisher라서 **inactive 상태여도** "연결됨, 스트리밍 안 함"을 보고합니다.

## 장치 정보

```bash
ros2 service call /scale_node/device_info hansung_scale_msgs/srv/DeviceInfo
```

```
device_name='Hansung HS-AA series RS232 indicator'
serial_number='FTEFY2BT'   physical_port='/dev/ttyUSB0'
port_id='/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTEFY2BT-if00-port0'
usb_type_descriptor='0403:6001'   serial_settings='2400 8N1'
connected=True   frame_rate=5.0   frames_received=1284   parse_errors=0
```

## USB 케이블을 뽑았을 때

`reconnect_timeout` 초 동안 계속 재시도합니다. **노드는 죽지 않고**, 어댑터가 돌아오면 자동으로 다시 붙습니다.

```
[ERROR] Serial read error: device reports readiness to read but returned no data
[WARN]  Link down; retrying for up to 6.0s...
[ERROR] No serial port matched [serial_no=FTEFY2BT] within 6.0s.
        `ros2 run hansung_scale_driver scale_sniffer --ros-args -p list_ports:=true` shows
        what is actually connected.
[WARN]  Link down; retrying for up to 6.0s...
[INFO]  Reconnected to /dev/ttyUSB0
```

`reconnect_timeout: -1.0`으로 두면 첫 읽기 에러에서 리더를 멈춥니다.

---

# 문제 해결

| 증상 | 원인 / 확인 |
|---|---|
| `Permission denied` | `dialout` 그룹. [1번 항목](#1-시리얼-포트-권한) |
| `No serial port matched` | 어댑터 미연결 또는 선택자 오타. `-p list_ports:=true`로 확인 |
| 토픽이 조용함 | lifecycle 상태 확인 (`ros2 lifecycle get`). `inactive`면 발행 안 함 |
| 값이 안 오는데 노드는 정상 | `~/raw`를 보세요. 라인은 오는데 파싱이 안 되는 건지, 아예 안 오는 건지 갈립니다 |
| `~/raw`가 깨진 문자 | baud/parity 불일치. `scale_sniffer`로 1200/2400/4800/9600을 훑어보세요 |
| `parse_errors`가 늘어남 | 프레임 포맷이 다릅니다. `-p decode:=true`로 파서 해석을 확인 |
| `ros2 param set`이 거부됨 | configure 시점 전용 파라미터. `cleanup` → `configure` |
| tare/zero가 안 됨 | 이 저울은 입력을 못 받습니다. 물리 버튼을 쓰세요 |

## 스니퍼로 원인 찾기

```bash
# raw 바이트 HEX/ASCII 덤프
ros2 run hansung_scale_driver scale_sniffer --ros-args -p serial_no:=FTEFY2BT

# 덤프 + 현재 파서가 그 라인을 어떻게 해석하는지 같이 보기
ros2 run hansung_scale_driver scale_sniffer --ros-args -p serial_no:=FTEFY2BT -p decode:=true
```

```
HEX[57 54 53 54 2b 20 20 31 32 2e 37 30 ...] ASCII['WTST+  12.70   g\r\n']
  LINE 'WTST+  12.70   g' -> HS-AA frame: weight=12.7 unit='g' stable=True status='ST' grams=12.7
```

`decode:=true`는 `line_ending`/`value_regex` 후보를 params 파일에 넣기 **전에** 검증하는 용도입니다.

---

# RS232 단방향 확인 내역

**제조사 확인:** 이 인디케이터의 RS232는 단방향 출력 전용. 데이터 읽기만 가능.

이 결론에 도달하기 전에 실기로 확인한 내역입니다. 나중에 같은 조사를 반복하지 않도록 남깁니다.

물건(12.70 g)을 올려둔 안정 상태(베이스라인 spread `0.00`)에서 **43종**을 던지고 네 가지 신호를 동시에 감시했습니다.

| 감시 신호 | 의미 |
|---|---|
| 값이 0으로 붕괴 | 타레/영점 성공 |
| 베이스라인 흔들림을 넘는 값 이동 | 뭔가 일어남 |
| 프레임 주기 저하 / 스트림 중단 | 저울이 처리하려고 멈춤 |
| `WT...` 아닌 라인 | 에러 / ACK 응답 |

| 축 | 던진 것 |
|---|---|
| 커맨드 바이트 (25종) | `T`/`t`/`T\r\n`/`T\r`, `Z`/`z`/`Z\r\n`/`Z\r`, `Q`, `S`, `SI`, `W`, `P`, `R` (각 bare/CRLF), ENQ `05`, ACK `06`, DC1, DC2, bare CR, bare LF |
| 프레이밍 (11종) | `WTT`/`WTZ`/`WT`, 주소 접두 `01T`/`00T`/`1T`, 어텐션 `@T`/`#T`/`*T`, 단어 `TARE`/`ZERO` |
| 핸드셰이크 라인 (7종) | RTS low, DTR low, RTS+DTR low 각각에서 `T`/`Z`, CRLF 유무, 연속 2회 |

**결과: 43종 전부 무반응.** 저울 표시창에도 아무 변화 없음. 모뎀 입력 라인(`CTS`/`DSR`/`DCD`)도 전부 low로, 3선 케이블로 보입니다.

> 교정·설정 모드에 진입할 수 있는 ESC 시퀀스와 `C` 계열은 **의도적으로 제외**했습니다. 전원을 뽑아야 빠져나오고, 최악의 경우 교정값이 흐트러집니다.

---

# 테스트

```bash
colcon test --packages-select hansung_scale_driver
colcon test-result --verbose --test-result-base build/hansung_scale_driver
```

**136개 테스트 전부 저울 없이 돌아갑니다.** 프레임 파싱과 포트 선택 로직은 순수 함수라 그대로 테스트하고, 실제 시리얼 fd가 필요한 부분은 `os.openpty()`로 만든 pty를 씁니다.

| 파일 | 대상 |
|---|---|
| `test_protocol.py` | 프레임 파싱, 단위 환산, 이스케이프 디코딩, 라인 조립 |
| `test_device.py` | 라인 설정 검증, 포트 선택 우선순위, 장치 대기 |
| `test_connection.py` | pty 상대로 실제 open/read/write/close |
| `test_scale_node.py` | 선언된 파라미터, 광고되는 토픽·서비스, 동적 파라미터 |
| `test_launch_args.py` | launch 파라미터 테이블 정합성, 타입 변환 |
| `test_flake8.py` / `test_pep257.py` | 린트 |

---

# 패키지 구성

```
cho_sensor/hansung_scale/
├── hansung_scale_driver/
│   ├── hansung_scale_driver/
│   │   ├── scale_node.py    # lifecycle 드라이버 노드
│   │   ├── raw_sniffer.py   # 포트 조회 + raw 덤프 + 파싱 미리보기
│   │   ├── protocol.py      # 프레임 파싱 (ROS/pyserial 비의존)
│   │   ├── device.py        # 포트 탐색·선택, 시리얼 설정, 연결 관리
│   │   └── qos.py           # QoS 프리셋 문자열 → QoSProfile
│   ├── config/scale_params.yaml
│   ├── launch/scale.launch.py
│   └── test/
└── hansung_scale_msgs/              # WeightStamped, DeviceInfo, SendCommand
```

`protocol.py`와 `device.py`가 ROS를 모르는 게 핵심입니다. 덕분에 파싱과 포트 선택을 저울도 ROS 그래프도 없이 테스트할 수 있고, 프로토콜이 다른 장비를 붙일 때 건드릴 파일이 하나로 좁혀집니다.

## realsense-ros 대응표

| realsense2_camera | hansung_scale_driver 대응 |
|---|---|
| lifecycle 노드 | 동일 |
| `serial_no` / `usb_port_id` / `device_type` 장치 선택 | 동일 (USB-시리얼 어댑터 기준) |
| `wait_for_device_timeout` / `reconnect_timeout` | 동일 |
| `enable_<stream>` / `<stream>_qos` | `enable_*` / `weight_qos`, `raw_qos` |
| `/diagnostics` (온도, 스트림 주기) | `/diagnostics` (링크 상태, 프레임 주기, 파싱 실패) |
| `~/device_info`, `~/hw_reset` | 동일 |
| 하드웨어 모니터 커맨드 서비스 | `~/send_command` (`enable_commands` 필요) |
| `rs_launch.py`의 `configurable_parameters` | `scale.launch.py` 동일 구조 |
| `realsense2_camera_msgs` | `hansung_scale_msgs` |

---

# 다른 인디케이터에 맞게 바꾸려면

1. `scale_sniffer -p list_ports:=true`로 포트를 찾고, `-p decode:=true`로 실제 프레임 포맷을 확인합니다.
2. `test_protocol.py`에 캡처한 라인을 케이스로 추가합니다. **장비 없이** 파서를 맞출 수 있습니다.
3. `hansung_scale_driver/protocol.py`의 `HS_AA_FRAME` 정규식을 새 포맷에 맞게 바꾸거나, 프레임 매칭 실패 시 자동으로 쓰이는 `value_regex` 파라미터만 조정합니다.
4. `config/scale_params.yaml`의 `baudrate`/`parity`/`expected_frame_rate`를 실측값으로 맞춥니다.
5. 그 장비가 입력을 받는다면 `enable_commands: true`로 켜고, `~/send_command`로 커맨드 바이트를 찾아 `tare_command`/`zero_command`에 넣습니다. 요청/응답형이면 `poll_mode: true` + `poll_command`를 씁니다 (이때 `timeout`은 `poll_interval`보다 작게).
