# Go2 on SpacemiT K3

## 直接运行

以下命令在 K3 板端以 root 执行，SDK 位于 `/root/spacemit_robot`，使用 `new_bigcore` 配置。
尚未安装模型或配置本机标定时，先完成[首次准备](#首次准备)。

**每次启动前，先摆好下图的蹲姿（Pose A），保持到程序完成启动。开机自启时，上电前就摆好。**
已启用自启服务时，手动启动前先执行 `systemctl stop go2.service`。

```bash
cd /root/spacemit_robot
bash application/ros2/unitree-go2/scripts/start-go2.sh
```

![Go2 启动蹲姿 Pose A](tool/calibration/assets/robot_replication/pose_a_overview.jpg)

程序默认使用当前 Wi-Fi。手机或 PC 连接同一网络，打开[控制网页](http://go2-sdk.local:8082/)，
在右上角选择机器狗。也可使用 `http://板端IP:8082/`，或安装 [Android App](clients/android/go2-web.apk)。
网页中的“打开摄像头”按钮用于预览。

停止时先按遥控器 **H 高 + D**，待机器狗蹲下并进入被动后，在启动终端按 `Ctrl+C`。

## 热点连接

先停止当前 Go2 程序，再临时以热点模式启动；后台运行可避免切换网络时 SSH 断开导致程序退出：

```bash
cd /root/spacemit_robot
nohup env GO2_NETWORK_MODE=ap bash application/ros2/unitree-go2/scripts/start-go2.sh \
  >/tmp/go2-ap.log 2>&1 </dev/null &
echo $! >/tmp/go2-ap.pid
```

热点名称由本机配置的 `robot.ap_ssid` 决定，参考配置为 `Go2`，默认密码为 `go2control`。
连接后打开[热点控制网页](http://10.0.91.250:8082/)。

停止临时热点程序时，在板端执行 `kill -INT "$(cat /tmp/go2-ap.pid)"`，程序会尝试恢复原来的 Wi-Fi。
需要长期使用热点时，安装下面的自启服务并切换到热点模式。

## 开机自启

先停止手动运行的程序，摆好 Pose A，再安装并启动服务。
SDK 路径不同时，先修改服务文件中的 `SROBOTIS_ROOT`、`WorkingDirectory` 和 `ExecStart`。

```bash
cd /root/spacemit_robot
install -m 644 application/ros2/unitree-go2/config/systemd/go2.service /etc/systemd/system/go2.service
systemctl daemon-reload
systemctl enable --now go2.service
```

服务默认使用 Wi-Fi；在 SDK 根目录按需执行以下命令，保存网络模式并重启服务：

| 网络模式 | 命令 |
| --- | --- |
| 热点 | `bash application/ros2/unitree-go2/scripts/start-go2.sh --switch ap` |
| Wi-Fi | `bash application/ros2/unitree-go2/scripts/start-go2.sh --switch wifi` |

| 日常管理 | 命令 |
| --- | --- |
| 查看状态 | `systemctl status go2.service` |
| 查看日志 | `journalctl -u go2.service -f` |
| 停止程序 | `systemctl stop go2.service` |
| 启动程序 | `systemctl start go2.service` |
| 取消自启并停止 | `systemctl disable --now go2.service` |

## 通过串口更换 Wi-Fi

将 K3 调试串口（UART0 / Console）接到 Linux PC。Go2 定制板按丝印与 USB 转串口模块同名连接：
**RX 接 RX、TX 接 TX、GND 接 GND**。

1. 在 PC 上查找设备并打开串口，将 `/dev/ttyUSB0` 替换为实际设备：

   ```bash
   ls -l /dev/serial/by-id/
   sudo minicom -D /dev/ttyUSB0 -b 115200 -8
   ```

   未安装 minicom 时执行 `sudo apt install minicom`。按 `Ctrl+A`、`O` 进入串口设置，
   使用 `115200 8N1`，关闭硬件和软件流控。按回车后登录，默认用户名 `root`、密码 `bianbu`；
   修改过密码则使用新密码，输入密码时不回显。

2. 在板端串口终端停止 Go2。服务方式执行 `systemctl stop go2.service`；
   手动运行则先停止对应程序。扫描并连接新 Wi-Fi：

   ```bash
   nmcli radio wifi on
   nmcli device wifi rescan
   nmtui connect
   ```

   用方向键选择 Wi-Fi，按 `Tab` 切到“激活”，输入密码。连接成功后退出。
   若列表没有刷新，退出 nmtui，重新扫描后再打开。

3. 查看连接名，设置自动连接，再查询 IP：

   ```bash
   nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
   nmcli connection modify "Wi-Fi 连接名" connection.autoconnect yes connection.autoconnect-priority 100
   ip -br addr
   ```

   将“Wi-Fi 连接名”替换为已连接无线网卡的 `CONNECTION` 值。
   `/etc/unitree-go2/robot.yaml` 中的 `robot.wifi_connection` 留空，程序就会沿用当前连接；
   若原来指定了旧网络，只清空这一项，保留其他配置。

4. PC 连接同一 Wi-Fi，在新终端用查到的 IP 登录，例如：

   ```bash
   ssh root@192.168.1.80
   ```

   摆好 Pose A 后，按[直接运行](#直接运行)启动；使用自启服务时，执行上一节的 Wi-Fi 切换命令。
   随后即可断开串口，用网页或 App 控制。

## 遥控器

左摇杆控制前后和横移，右摇杆左右控制转向。
B 保持暗，使用 F、E 选择常用模式；H 置中后按 C 进入：

| F | E | 模式 |
| --- | --- | --- |
| 低 | 低 | 综合地形 |
| 低 | 中 | 招手 |
| 低 | 高 | 坐下 |
| 中 | 低 | 网页控制，无需按 C |
| 高 | 低 | 前跳 |
| 高 | 高 | 平地行走 |

| 操作 | 按键 |
| --- | --- |
| 站立 / 蹲下 | G 高 / 低保持 0.5 秒 |
| 正常休眠 | H 置中后按 D，完成当前动作的退出过程后蹲下并进入被动 |
| 紧急制动 | H 高 + D，不等待动作退出，直接蹲下并进入被动 |
| 自起 | 被动或蹲下时，H 低保持 0.5 秒 |
| 白灯开关 | H 高 + C |

网页档位下由网页控制，遥控器仅保留急停和白灯操作；切出后恢复遥控器控制。
关闭遥控器或持续断连也会交给网页。G 控制的蹲下与模式表中的坐下动作不同。

## 终端控制

保持 Go2 程序运行，另开一个板端终端，加载环境和本机通信域：

```bash
cd /root/spacemit_robot
source build/envsetup.sh
sros2_setup
go2_config="${GO2_LOCAL_CONFIG:-/etc/unitree-go2/robot.yaml}"
export ROS_DOMAIN_ID="$(python3 -c 'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["robot"]["ros_domain_id"])' "$go2_config")"
```

例如切换到站立：

```bash
ros2 service call /go2/set_mode unitree_go2/srv/SetMode '{mode: stand}'
```

将 `stand` 替换为 `terrain`（综合地形）、`crouch`（蹲下）、`passive`（正常休眠）
或 `passive_direct`（紧急制动），即可执行对应操作。

## 首次准备

应用放在 SDK 的 `application/ros2/unitree-go2`。在 K3 上下载模型并构建：

```bash
cd /root/spacemit_robot
python3 application/ros2/unitree-go2/scripts/download_models_go2.py
source build/envsetup.sh
lunch k3-com260-unitree-go2
m
```

本机身份和标定保存在 `/etc/unitree-go2/robot.yaml`。已有配置继续使用；
首次部署时，在 SDK 根目录执行以下命令，仅在文件不存在时复制模板：

```bash
test -e /etc/unitree-go2/robot.yaml ||
  install -D -m 600 application/ros2/unitree-go2/config/robot.yaml /etc/unitree-go2/robot.yaml
```

参照 [config/robot.yaml](config/robot.yaml)，将 `robot.board_profile` 设置为 `new_bigcore`，
填写本机身份、通信域和 IMU 补偿，再按 [XYZ 标定说明](tool/calibration/CALIBRATION.md) 完成电机标定。
模板的标定数据来自样机，不能直接套用到其他机器，完成本机标定后再启动。

摄像头预览需要 FFmpeg 和 [MediaMTX riscv64](https://github.com/Fitz8863/spacemit-mediamtx/releases/tag/v1.20.1-riscv64)，
将 `mediamtx` 放入 `/usr/local/bin/`。

## 许可

源码和项目自训模型采用 Apache-2.0，第三方综合地形模型采用 MIT。
模型来源、版本和校验值见 [config/policies.yaml](config/policies.yaml) 的 `model_manifest`，
版权声明和许可原文见 [NOTICE](NOTICE) 与 [LICENSE](LICENSE)。
