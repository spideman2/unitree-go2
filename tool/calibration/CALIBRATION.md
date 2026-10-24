# Go2 关节标定

本工具通过 K3 直连电机的 BRAKE 反馈计算零偏。直接运行进入 Pose X/Y/Z 终端交互，
依次采集、计算并预览结果；最后输入“确认标定”才写入本机配置。也支持分阶段采集和离线合并。

## 终端交互标定

先停止 Go2 主程序及其他电机串口程序，支撑好机身。在构建安装后的板端执行：

```bash
cd /root/spacemit_robot
systemctl stop go2.service
source output/staging/setup.bash
python3 output/staging/lib/unitree_go2/tool/calibration/calibrate_robot.py
```

脚本依次提示摆到 Pose X、Y、Z。每次摆好后按回车，倒计时 **5 秒**才开始采集；
记录期间保持姿势。某阶段采样不稳定或反馈异常时，会提示重摆并重采该阶段。
串口启动时首帧超时会最多重试 3 次，成功后仍采满 20 帧新的有效反馈；持续超时或异常反馈会中止并记录具体错误。
三组采集并校验完成后，终端提示可以松手、放稳机器狗，然后显示 12 个关节的新旧零偏
和 Pose A 转子参考。

原始采样及 `calibrated.yaml` 默认保存在 SDK 的 `output/calibration/poseXYZ-<时间>/`。
最后输入“确认标定”才更新 `/etc/unitree-go2/robot.yaml` 中的零偏和启动转子参考，其他
本机配置保持原值；直接回车仅保留结果，不应用。摆姿阶段输入 `q` 或按 `Ctrl+C` 可退出。
确认后的标定在下次启动生效，启动前恢复 Pose A。

`GO2_LOCAL_CONFIG` 或 `--base-config` 可指定本机 YAML。需要指定新的采集目录时使用
`--interactive <目录>`，`--output <新文件>` 可指定结果文件；已有结果不覆盖。

## poseX / poseY / poseZ：三阶段标定

三个姿势必须分别摆放和采集，每次只采该阶段对应的电机。
参考角度和均值规则保存在 `calibration_pose_xyz.yaml`。

poseX 标定四条腿的大腿和小腿，共 8 个电机。前腿大腿、小腿各使用左右两腿读数绝对值的
均值，后腿同样分别取均值；均值作为左右同组关节的目标角。

![poseX 标定姿势](assets/robot_replication/pose_x_overview.png)

poseY 标定左前和左后髋关节，共 2 个电机；poseZ 标定右前和右后髋关节，共 2 个电机。
前髋参考角对 poseY 左前与 poseZ 右前取绝对值均值 43.30°，左侧记正、右侧记负；后髋
同理取均值 48.96°。poseZ 图中左侧髋角是未参与该阶段的读数。

![poseY 标定姿势](assets/robot_replication/pose_y_overview.png)

![poseZ 标定姿势](assets/robot_replication/pose_z_overview.png)

关节顺序为 FR、FL、RR、RL，每条腿依次为 hip、thigh、calf；角度单位为度。髋和大腿
减速比为 6.33，小腿为 12.13，方向沿用目标机器人基础配置。

## 非交互采集和合并

先停止 Go2 主程序及其他电机串口程序，确保总线上只有 K3 一个主站。机身可靠支撑并摆到
对应参考姿势，采样期间保持不动。每次采集 20 帧 BRAKE 反馈；每关节采样跨度须不超过
0.1°。三个采集目录必须使用新路径。在当前板端执行：

```bash
cd ~/spacemit_robot
source /opt/ros/humble/setup.bash
source ~/spacemit_robot/output/staging/setup.bash
python3 output/staging/lib/unitree_go2/tool/calibration/calibrate_robot.py /tmp/go2_poseX \
  --phase poseX --capture
python3 output/staging/lib/unitree_go2/tool/calibration/calibrate_robot.py /tmp/go2_poseY \
  --phase poseY --capture
python3 output/staging/lib/unitree_go2/tool/calibration/calibrate_robot.py /tmp/go2_poseZ \
  --phase poseZ --capture
python3 output/staging/lib/unitree_go2/tool/calibration/calibrate_robot.py \
  --output /tmp/go2_calibrated.yaml \
  /tmp/go2_poseX /tmp/go2_poseY /tmp/go2_poseZ
```

执行每阶段采集时，只启动 BRAKE 采样工具，不启动控制节点；命令会检查四个串口均空闲。
采样目录保存基础配置、姿势参考和原始反馈。合并时验证三组采样来自同一基础配置和参考，
只替换对应 12 个关节零偏及启动转子参考，其他配置不变。
采集时可用 `--base-config` 显式提供本机配置，`--reference` 指定 XYZ 参考文件。
PC 上可使用源码目录的 `tool/calibration/calibrate_robot.py` 执行离线合并，需要 PyYAML。

上述非交互命令输出独立试验配置，不会自动替换机器人配置。检查结果后再通过启动参数
`local_config:=/tmp/go2_calibrated.yaml` 加载；真机启动前恢复基础配置定义的
Pose A。Pose X/Y/Z 是标定姿势，不是运行启动姿势。

Pose A 是日常上电的蹲姿，用于建立本轮上电的转子位置锚点：

![Pose A 启动姿势](assets/robot_replication/pose_a_overview.jpg)
