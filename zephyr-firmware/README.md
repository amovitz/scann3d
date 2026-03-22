# scann3d

## Zephyr

| Library | Version |
|---|---|
| west | 1.5.0 |
| Zephyr | 4.3.0 |
| Tollchain | 0.17.3 |
| VL53L8CX ULD | 1.0.0 |

## Hardware - ESP32-C3-WROOM-02-N4

| Device | Part | Bus | Address | Notes |
|---|---|---|---|---|
| ToF sensor | VL53L8CX | I2C0 | 0x29 | I2C_RST - IO8 |
| Main IMU | LSM6DSV(TR) | I2C0 | 0x6A | On-Board |
| Tracker IMU | LSM6DSV(TR) | I2C1 | 0x6B | Detachable |

### Pin map (edit `boards/esp32c3_devkitc.overlay` to change)

```
IO0   LED1-B
IO1   LED1-G
IO2   LED1-R
IO3   LED2-B
IO4   LED2-G
IO5   LED2-R
IO6   I2C0 SDA
IO7   I2C0 SCL
IO8   VL53L8CX I2C_RST (active-low)
IO9   Servo PWM
IO10  RFU
IO18  USB D-  (native USB OTG - do not reassign)
IO19  USB D+  (native USB OTG - do not reassign)
IO20  RFU (Serial RXD)
IO21  RFU (Serial TXD)
```

Pull SA0 on the main LSM6DSV to GND -> address 0x6A.

Pull SA0 on the tracker LSM6DSV to VDD -> address 0x6B.

---

## Prerequisites

### Installing automatically 

I *highly* recommend installing the [Zephyr Extension for VSCode](https://marketplace.visualstudio.com/items?itemName=mylonics.zephyr-ide) and using it to install tools to the `zephyr-firmware` directory.

Once tools are installed, add a **Project** with the `esp32c3_devkitc` board.

Under **Project Config**, add the `boards/esp32c3_devkitc.overlay` to **zephyr-firmware/DTC Overlay**.

Then:

```bash
cd zephy-firmware
. .venv/bin/activate
west packages pip --install
west blobs fetch hal_espressif
```


### Installing manually

```bash
# Install west and the Zephyr SDK (>=0.16) first:
#   https://docs.zephyrproject.org/latest/develop/getting_started/index.html

pip install west

# Clone and initialise the workspace
cd zephy-firmware
west init
west update
west packages pip --install
west blobs fetch hal_espressif
west zephyr-export
```


### Building & Flashing manually

```bash
# Set your WiFi credentials (or edit prj.conf)
export SCANNER_WIFI_SSID="my_ap"
export SCANNER_WIFI_PSK="my_password"

west build -b esp32c3_devkitc ./ \
    -p \
    --build-dir build/esp32c3_devkitc \
    -- \
    -DCONFIG_SCANNER_WIFI_SSID=\"${SCANNER_WIFI_SSID}\" \
    -DCONFIG_SCANNER_WIFI_PSK=\"${SCANNER_WIFI_PSK}\"
    -DCONFIG_DEBUG_OPTIMIZATIONS=y \
    -DCONFIG_DEBUG_THREAD_INFO=y \
    -DEXTRA_DTC_OVERLAY_FILE='boards/esp32c3_devkitc.overlay;'

west flash
```

### VL53L8CX ULD (required - not bundled)

Download the driver from [ST Micro directly](https://www.st.com/en/embedded-software/stsw-img040.html).

[VL53L8CX ULD Reference Manual](https://www.st.com/resource/en/user_manual/um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf)

```bash
# Unzip files
unzip STSW-IMG040.zip

# Copy ULD API
cp STSW-IMG040/VL53L8CX_ULD_driver_2.0.1/VL53L8CX_ULD_API/src/* \
   STSW-IMG040/VL53L8CX_ULD_driver_2.0.1/VL53L8CX_ULD_API/inc/* \
   lib/vl53lcx_uld/

# Copy Platform
cp STSW-IMG040/VL53L8CX_ULD_driver_2.0.1/Platform/platform.c \
   STSW-IMG040/VL53L8CX_ULD_driver_2.0.1/Platform/platform.h \
   lib/vl53lcx_uld/
```

The ULD contains ST proprietary firmware; it cannot be redistributed here.

**Note:** Fix missing semicolon on line 368 of `vl53l8cx_api.c`

---

## Output streams

### USB serial (CDC-ACM)

Connect a USB cable. The device appears as `/dev/ttyACMx` (Linux) or
`COMx` (Windows). Baud rate is irrelevant - CDC-ACM is USB-native.

### UDP

Frames are sent to `CONFIG_SCANNER_UDP_HOST:CONFIG_SCANNER_UDP_PORT`
(default `192.168.1.100:5005`). The scanner holds a static IP
(`192.168.1.200`) by default; change `CONFIG_NET_CONFIG_MY_IPV4_ADDR`
in `prj.conf` or enable `CONFIG_NET_DHCPV4` to use DHCP instead.

### Raw serial

RFU

---

## JSON Protocol

TODO: Fill this section

## Raw Wire protocol

All frames share a 9-byte envelope:

```
[0x55][0xAA]  magic        2 bytes
[type]        packet type  1 byte  (0x01=ToF, 0x02=IMU0, 0x03=IMU1, 0x04=SER0, 0xFF=Status)
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
| PKT_SER0 (0x04) | 72 bytes | 81 bytes |
| PKT_STATUS (0xFF) | 5 bytes | 14 bytes |

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
int16_t  accel_{x,y,z}         // ±16 g,    LSB = 0.488 mg
int16_t  gyro_{x,y,z}          // ±2000 dps, LSB = 0.061 dps
int16_t  temp_raw              // °C = (raw / 256.0) + 25.0
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
If the connector is removed, `PKT_IMU1` frames simply stop. Re-inserting
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
