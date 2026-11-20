# 在 RDK X5 上复现

本指南面向安装 Ubuntu 22.04 的 RDK X5。记录实际镜像、Python 版本与设备树型号后，再比较不同部署结果。仓库当前状态为待板端验证。

## 1. 先打开离线仿真

在板上进入仓库，运行 `python3 scripts/doctor.py` 查看环境，再运行 `python3 server.py`。服务默认绑定回环地址。Desktop 镜像可以从板上的浏览器打开 <http://127.0.0.1:8766/>。使用 Server 镜像时，在电脑另开终端，把下方 `USER` 和 `BOARD_IP` 换成板子的实际账号和地址。

```bash
ssh -N -L 18766:127.0.0.1:8766 USER@BOARD_IP
```

电脑浏览器打开 <http://127.0.0.1:18766/>。此时板子负责 HTTP 服务，浏览器所在电脑承担 WebGL 和 WASM 计算。访问 `/health` 可查看 `simulation_only` 与 `hardware_execution_enabled: false`。先用默认偏移播放一次，应看到 F008、约 5.7 mm，约 57.3 秒后返回起点。

## 2. 接入固定 D455

将相机固定在桌面支架，通过 USB 3 接入目标主机。先确认 SDK 能枚举相机。在兼容该系统、Python ABI 和 CPU 架构的环境内安装 RealSense Python 绑定，再运行 `python -c 'import pyrealsense2 as rs; print(len(rs.context().query_devices()))'`，预期至少一台。已有系统级绑定时可以用 `python3 -m venv --system-site-packages .venv` 创建项目环境。确认环境中的绑定能导入后安装其他依赖。[RealSense Linux 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md)

```bash
. .venv/bin/activate
python -m pip install -r camera/requirements.txt
python scripts/fetch_vision_assets.py
python -m camera.realsense
```

首次下载需要联网并核对清单中的哈希。此后模型由板端 HTTP 服务发送到浏览器，无需推理云服务。相机程序默认仅在本机 5000 端口接收请求，页面服务使用同源代理访问它。相机进程与页面服务应分别在两个终端运行。

```bash
curl --fail http://127.0.0.1:8766/api/rgbd/status
```

检查 `ok`、`aligned_to=color`、帧号与帧龄。在页面点击「连接真实 RGB-D 相机」，单人正对相机并保持约一秒稳定，看到有效相机 XYZ 后点击「设置嘴部基准并开启」。先闭嘴再张嘴，观察轨迹选择。退出时分别在服务所在终端按 Ctrl+C。

## 3. 记录板端验收结果

| 验收项 | 建议记录 |
|---|---|
| 环境 | 板型、内存、镜像版本、Python 与 RealSense SDK 版本、浏览器运行在哪台设备 |
| 相机 | 彩色与深度流是否均为 640×480，连续帧号与帧龄，同帧查询成功比例 |
| 交互 | 单次张嘴是否只选一条，持续张嘴是否重复触发，移动或遮挡时是否暂停 |
| 延迟 | 至少一段连续采样内的采集帧龄、浏览器推理耗时与端到端 P50/P95，记录网络与浏览器位置 |
| 回放 | 时间线是否走完、轨迹编号与排名第一是否相符、勺面倾角及视频文件能否播放 |

浏览器在远端运行时，链路往返也会消耗 500 ms 的新鲜度预算。若页面长期显示「识别延迟过大」，优先检查相机帧龄、浏览器运行位置和网络。若返回 `insufficient_depth` 或 `depth_edge_or_unstable_surface`，检查嘴部遮挡、距离及深度边缘。未连接相机时，离线演示仍可独立使用。

后续 BPU 适配需要选定 X5 支持的关键点模型、记录模型转换版本，验证输出点序与坐标系，并用同一批输入对照 CPU 结果及延迟。[地瓜机器人 RDK 开发手册](https://d-robotics.github.io/rdk_x_doc/RDK/)
