# scann3d - ESP32-C3-WROOM-02-N4

## Hardware

| Device | Part | Bus | Address | Notes |
|---|---|---|---|---|
| ToF sensor | VL53L8CX | I2C0 | 0x29 | XSHUT → GPIO6 |
| Main IMU | LSM6DSV(TR) | I2C0 | 0x6A | INT1 → GPIO7 |
| Tracker IMU | LSM6DSV(TR) | I2C1 | 0x6B | INT1 → GPIO10, detachable |

### Pin map (edit `boards/esp32c3_devkitm.overlay` to change)

```
GPIO4   I2C0 SDA
GPIO5   I2C0 SCL
GPIO6   VL53L8CX XSHUT (active-low)
GPIO7   LSM6DSV #0 INT1
GPIO8   I2C1 SDA
GPIO9   I2C1 SCL
GPIO10  LSM6DSV #1 INT1
GPIO18  USB D-  (native USB OTG - do not reassign)
GPIO19  USB D+  (native USB OTG - do not reassign)
```

Pull SA0 on the main LSM6DSV to GND → address 0x6A.
Pull SA0 on the tracker LSM6DSV to VDD → address 0x6B.

---

## Prerequisites

```bash
# Install west and the Zephyr SDK (>=0.16) first:
#   https://docs.zephyrproject.org/latest/develop/getting_started/index.html

pip install west

# Clone and initialise the workspace
mkdir scanner-ws && cd scanner-ws
west init -l scanner/          # if you cloned this repo into scanner/
west update
west zephyr-export
```

### VL53L8CX ULD (required - not bundled)

```bash
git clone https://github.com/STMicroelectronics/VL53L8CX_ULD_driver \
          scanner/lib/vl53l8cx_uld
```

The ULD contains ST proprietary firmware; it cannot be redistributed here.

---

## Build & Flash

```bash
# Set your WiFi credentials (or edit prj.conf)
export SCANNER_WIFI_SSID="my_ap"
export SCANNER_WIFI_PSK="my_password"

west build -b esp32c3_devkitm scanner/ -- \
    -DCONFIG_SCANNER_WIFI_SSID=\"${SCANNER_WIFI_SSID}\" \
    -DCONFIG_SCANNER_WIFI_PSK=\"${SCANNER_WIFI_PSK}\"

west flash
```

---

## Output streams

### USB serial (CDC-ACM)

Connect a USB cable.  The device appears as `/dev/ttyACMx` (Linux) or
`COMx` (Windows).  Baud rate is irrelevant - CDC-ACM is USB-native.

### UDP

Frames are sent to `CONFIG_SCANNER_UDP_HOST:CONFIG_SCANNER_UDP_PORT`
(default `192.168.1.100:5005`).  The scanner holds a static IP
(`192.168.1.200`) by default; change `CONFIG_NET_CONFIG_MY_IPV4_ADDR`
in `prj.conf` or enable `CONFIG_NET_DHCPV4` to use DHCP instead.

---

## Wire protocol

All frames share a 9-byte envelope:

```
[0x55][0xAA]  magic       2 bytes
[type]        packet type  1 byte  (0x01=ToF, 0x02=IMU0, 0x03=IMU1, 0x10=Status)
[seq]         sequence     2 bytes LE - per-type rolling counter
[len]         payload len  2 bytes LE
[payload]     N bytes
[crc16]       CRC-16/CCITT 2 bytes LE  (over type+seq+len+payload)
```

### Packet sizes

| Type | Payload | Total frame |
|---|---|---|
| PKT_TOF (0x01) | 329 bytes | 338 bytes |
| PKT_IMU0/1 (0x02/03) | 20 bytes | 29 bytes |
| PKT_STATUS (0x10) | 5 bytes | 14 bytes |

### ToF payload (`tof_payload_t`)

64 zones in row-major order (zone 0 = top-left when sensor label faces up).

```c
uint32_t timestamp_ms
uint16_t distance_mm[64]       // centre target distance
uint16_t sigma_mm[64]          // ranging sigma
uint8_t  status[64]            // ULD target_status (5 = valid)
uint8_t  nb_target_detected[64]
```

### IMU payload (`imu_payload_t`)

```c
uint32_t timestamp_ms
int16_t  accel_{x,y,z}   // ±16 g,    LSB = 0.488 mg
int16_t  gyro_{x,y,z}    // ±2000 dps, LSB = 0.061 dps
int16_t  temp_raw         // °C = (raw / 256.0) + 25.0
```

---

## Sampling rates

| Sensor | Timer period | Effective rate |
|---|---|---|
| LSM6DSV (both) | 1 ms | up to 1 kHz drain; ODR = 3840 Hz internally |
| VL53L8CX | 66 ms | ~15 Hz (8×8 mode) |

Increase `CONFIG_SCANNER_TOF_PERIOD_MS` to 34 ms (~29 Hz) cautiously -
VL53L8CX firmware overhead over I2C at 400 kHz can exceed that budget at 8×8.

---

## Hot-plug tracker IMU

The tracker IMU (IMU1) is checked for presence every ~1024 IMU ticks (≈1 s).
If the connector is removed, `PKT_IMU1` frames simply stop.  Re-inserting
the connector will resume frames within ~1 s without a reboot.

---

## Python receiver example

```python
import socket, struct, sys

MAGIC  = b'\x55\xAA'
HDR    = 7   # magic(2)+type(1)+seq(2)+len(2)
FTR    = 2   # crc16

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5005))

while True:
    data, addr = sock.recvfrom(4096)
    if data[:2] != MAGIC:
        continue
    pkt_type, seq, length = struct.unpack_from('<BHH', data, 2)
    payload = data[HDR:HDR+length]

    if pkt_type == 0x01:   # ToF
        ts, = struct.unpack_from('<I', payload, 0)
        dist = struct.unpack_from('<64H', payload, 4)
        print(f"[ToF  seq={seq}] ts={ts}ms  centre={dist[27]}mm")

    elif pkt_type in (0x02, 0x03):  # IMU
        ts, ax, ay, az, gx, gy, gz, tmp = struct.unpack_from('<IhhhhhHh', payload)
        label = 'IMU0' if pkt_type == 0x02 else 'IMU1'
        print(f"[{label} seq={seq}] ts={ts}ms  a=({ax},{ay},{az})  g=({gx},{gy},{gz})")
```
