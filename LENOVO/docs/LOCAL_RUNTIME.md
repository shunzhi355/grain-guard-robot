# 联想直连：完整取样流程迁移与验收

## 已确认环境与边界

2026-10-10 通过 SSH 只读确认：联想 `192.168.1.200`，用户 `ps02`，Ubuntu 22.04.5、x86_64、Python 3.10.12；实际雷达/定位为 Docker 内 ROS Noetic。主机上虽有 `/opt/ros/humble` 目录，但没有 `setup.bash`，不能当成已装可用的 ROS2。已有独立雷达任务正在运行，不应重启、覆盖或接管。

2026-10-10 后续只读枚举已发现两只转换器，用户确认新增的 FT232R 接 STM32，原 FT231X 接 X2P。已将下列稳定身份写入 `LENOVO/config/local_robot.json`：

| 用途 | 用户标注的插口 | 转换器序列号 | 检测时设备 |
| --- | --- | --- | --- |
| STM32 USB 串口 | USB4 | FT232R `AB3N91KX` | `/dev/ttyUSB1` |
| X2P USB-RS485 | USB3 | FT231X `D30GLD5V` | `/dev/ttyUSB0` |

实际配置使用 `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0` 和 `/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30GLD5V-if00-port0`，不依赖可能变动的 `ttyUSB` 编号；机箱插口标注不是内核端口编号。更换转换器或改变下游接线后必须重新核对，不能只看芯片型号。

此次仅绑定配置并检查设备节点，未打开串口、未启动实机服务；`navigation_mode=operator`、`suction_policy=required` 和 `acknowledge_legacy_untighten=false` 保持不变。不得标为机构动作、底盘、急停线路或 RS485 稳定性实测通过。

代码中的 `3588/` 只是共享源码目录，不是网络依赖。整包放在联想，本机 Python 导入共同 UI、状态机、驱动；运行时不访问 3588 IP，也不启动 GRICP 或 TLS 服务。

```text
联想原 UI / 完整取样编排
          │ 本机 control.sock，权限 0600
联想 LocalRobotService（唯一串口所有者）
          ├── USB-TTL / USB串口 → STM32 → 底盘、夹爪、三仓、输粮等
          └── USB-RS485 → X2P → 伺服升降

联想 Noetic 容器：定位/点云 → 本机导航适配器 → 同一个 control.sock
```

## 保留的完整流程

原 `SamplingStateMachine` 与 `WorkflowOrchestrator` 继续负责：记录起点、到点、接管确认、夹紧、往复下压、松夹等待、绝对回程、再夹紧、加管拧紧、废粮确认、浅中深仓分别取样、定时输粮/关仓/停粮、取管下降和上提、人工托管确认、拧松/松夹、逐节移除、下一点及返航。

迁移没有自动跳过接管、加管、托管、移除管节等人工确认。`workflow-sim` 才会自动确认这些提示并加速等待，且报告明确标记为模拟；真实 UI 不会自动确认。

机构串口序列完全复用原 `MechanismRuntime`。仅取消其 PCA9685 镜像输出。升降直接复用原校准的 `move_lift`，保留中断不重放、松夹前禁止回升、取管准备及绝对原点恢复逻辑。持久化的 `lift-recovery.json` 在可能运动前写入，进程崩溃后也要求人工机械复位确认，不默认丢弃未完成升降。

STM32 机构动作的可靠帧若 0.4 秒内未收到确认，联想最多发送三次**同一会话、同一序号**的请求；首帧若未到达可由重试补送，首帧若已执行则由现行固件的缓存回同一结果，不重复触发定时动作。每次发送前重新核验串口、固件启动号、遥控自动档和急停；重启、重枚举、拒绝或三次确认仍失败均锁停，不换序号自动重放。现场固件若与本仓库的去重实现不一致，必须先验收重复帧行为，不能把模拟通过当作实机保证。

### 两个必须说明的原串口边界

- 拧松保持原 `STOP TIGHTEN` 指令，没有新编单片机命令；配置 `acknowledge_legacy_untighten=true` 表示现场明白并验收该行为，不等于软件已经证明反转。
- 原串口流程没有独立风机启停 opcode。默认 `suction_policy=required` 会在任务开始前阻止完整流程。只有确认风机由现行固件联动/独立控制或由现场负责时，才可配置 `external` 并使用 `--external-suction-confirmed`。日志保留 `external_suction_required`，不声称发送了不存在的指令。若实际必须由上位机单独启停风机，则仍须提供正确接口，不能靠此配置完成自动风机控制。

## 安装与无硬件验证

本次已在联想创建独立验证目录 `/home/ps02/grain_robot/staging/lenovo-local-20261010`，安装好专用 `.venv`。直接进入该目录使用 `.venv/bin/python` 即可，不必重新安装。没有覆盖 `/home/ps02/grain_robot/source/` 的旧部署，也没有注册自启动服务或留下运行中的测试守护进程。

从发布包根目录执行。建议专用 venv，避免改动现有雷达容器或系统 Python：

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r LENOVO/requirements-local.txt
.venv/bin/python LENOVO/scripts/local_robot.py inventory
.venv/bin/python LENOVO/scripts/local_robot.py check
.venv/bin/python LENOVO/scripts/local_robot.py workflow-sim \
  --runtime-dir "$PWD/validation/full-flow" \
  --report "$PWD/validation/full-flow/report.json"
.venv/bin/python -m pytest -o addopts= -q LENOVO/test
.venv/bin/python LENOVO/scripts/check_local_stack.py --output validation/ui-stack
```

该联想宿主机缺少 venv 的 ensurepip，实际采用 `python3 -m venv --without-pip --system-site-packages .venv`，然后用 `.venv/bin/python -m pip` 安装依赖到此虚拟环境。PyPI 默认源在本次网络中出现 TLS EOF，改用 HTTPS 清华镜像成功；未关闭证书校验、未修改系统 Python。已解析的依赖版本见部署目录 `validation/requirements-resolved.txt`。

完整流程模拟使用两个点位，每点三档深度 `[1.5,2.5,3.5]`，覆盖加管与拆管分支。没有连接 ROS、硬件或云服务；报告含状态转移、模拟人工确认、STM32 字节帧、X2P Modbus 字节帧及故障日志。报告文件不覆盖同名文件。

`check_local_stack.py` 是独立的联想 Linux 端到端检查：启动临时的 **模拟** 服务子进程，打开真实主窗口进行离屏渲染，经本机 Socket 验证创建任务、档位显示、人工到位按钮、关闭窗口停止及进程退出。测试禁止物理串口、PCA9685、摄像头、外部网络，不写入实际任务记录；不自动运行实机流程。

Linux 上也可启动模拟硬件守护进程，检查 UI 与真实本机 IPC：

```bash
.venv/bin/python LENOVO/scripts/local_robot.py serve --simulate
# 另一个终端：仅检查 Qt 导入，不创建任务
QT_QPA_PLATFORM=offscreen .venv/bin/python LENOVO/scripts/local_robot.py ui --check-qt
```

默认配置阻止缺失风机处理的完整任务。模拟 UI 走全流程时，也需使用单独的配置副本将 `suction_policy` 改为 `external`，与实机配置分开。不要删除真实 `lift-recovery.json` 来绕过复位。

## 实机机构完整流程（第一阶段，底盘不自动走）

1. 确认两适配器已连接，运行 `inventory`，通过拔插识别哪个连接 STM32、哪个连接 X2P；这里只枚举设备，不打开串口。
2. 复制并检查 `LENOVO/config/local_robot.json`：当前两只转换器的 `/dev/serial/by-id/...` 已按用户确认填写；更换设备时更新为 **两个不同的** 稳定路径，不要用可能重排的 `/dev/ttyUSB0/1` 猜测。确认从站地址、保持现有升降方向与标定。默认转速 30 rpm，未自动提高。
3. `navigation_mode` 保持 `operator`。这个模式从未授权底盘自动运动；UI 在到点与返航阶段显示“确认静止到位”，按下后才继续机构流程。
4. 根据上述边界确认拧松、风机处理。现场有人监护、行程清空、硬件急停及垂直轴防坠有效后，才启动：

```bash
.venv/bin/python LENOVO/scripts/local_robot.py serve \
  --config /absolute/path/live.json --live --attended --external-suction-confirmed

# 联想图形桌面的另一个终端：
.venv/bin/python LENOVO/scripts/local_robot.py ui --config /absolute/path/live.json
```

UI 可创建本地任务，先用少点位、少深度走通，再做三仓、多点与连续循环。不要因为模拟完成就直接采用长行程/高负载。验证中记录每个实际动作与 UI 状态的对应关系、编码器位置、串口错误和系统 USB 重枚举事件。

UI 每 100 ms 向本机服务送存活状态。活跃任务丢 UI/流程存活约 1.5 s 后锁存停止；窗口正常关闭也主动急停。底盘速度仍有独立 200 ms 更新超时，叠加线程与 I/O 延迟，不能当作硬实时安全承诺。掉线后不自动恢复流程。断链时停止无法确认会记录日志，必须用硬件急停独立保障。

联想的“故障复位”不以 CLEAR_ESTOP 串口写入成功作为完成：守护进程必须在发送后收到同一 STM32 启动周期的新状态帧，确认急停位清除、故障为 0 且自动档有效，才释放本机锁存；未确认则保持锁定并提示复位未确认。即使当前 MCU 状态已正常，先前的本机急停锁也不能自动消失，仍需现场检查后由操作员显式复位；不会继续旧任务。

联想升降已按 3588 的执行策略调整：**每个机构动作开始前**仍要求 STM32 在线、遥控自动档有效且底盘未授权行驶；X2P 位置段已经启动后，不再因遥控档位/新鲜度或 STM32 的 `FAULT_RC` 位取消该段。运行中仍检查 STM32 串口重枚举、重启、已收到的急停及非遥控故障、底盘误授权、UI 心跳，并保留 X2P 自身停机和不自动重放逻辑。代价是遥控信号单独丢失不会立刻停止当前升降段，现场必须验证独立硬件急停和垂直轴防坠；这项策略不用于导航，导航自动档授权及速度超时停车保持原样。

## 本机导航与雷达（第二阶段）

`local_navigation_ros1.py` 只读取 ROS 数据并通过本机 Unix Socket 请求动作，不打开第二份串口，不经过 3588。接口需要：

- 实际定位 odometry 话题，正确 `map_frame`；不能把不相干 frame 假称为 map。
- 正确机体坐标系的点云（`body` 或 `base_link`）；空点云、错误坐标系、过期点云不作为“道路畅通”。
- 真实的定位质量 `Bool` 心跳，默认 `/grain/localization_valid`，更新间隔需小于 0.5 s。**不能为了跑起来恒定发布 true**；现有定位任务未提供该信号时，自动导航保持不可用。
- 配置与本地任务一致的 `map_id`，并将 `navigation_mode` 改为 `local`。

在具有 ROS Noetic Python 依赖的专用容器/进程中运行，按相同绝对路径只挂载发布源码和本机运行目录；不要重启当前正在采集的容器。该进程不需要映射 `/dev`：

```bash
python3 LENOVO/scripts/local_navigation_ros1.py \
  --config /absolute/path/live.json --allow-point-navigation
```

此适配器是复用既有控制原则的**点位直达联调控制器**，不是 Nav2 路径规划或绕障能力实现。障碍出现即停止并锁存，不恢复旧目标。正式复杂粮仓导航仍须接入并验收实际规划器；不能把点位到达测试当成全场景自主导航交付。

同机控制使用 monotonic 时钟，不需要两主机对时。导航动作必须有有效目标、当前 epoch、实时定位/点云/遥控状态。速度过期、重放序号、旧目标、掉进程、定位失效都会阻止继续运动。到达前需要三帧零速和实际位置/朝向误差校验。

点云预览通过本机 `preview.json` 传递，UI 读取有时效、点数与大小上限的本地文件，不进行双机地图传输。UI 建图选择 `LocalSlamBridge`，不再使用远程地图代理。宿主机没有 ROS1 环境时，它会明确拒绝建图管理；需要在已配置好 Noetic/FAST-LIO 工作空间的专用 UI 环境中启用，不能接管正在运行的独立采集任务。地图保存路径可用 `SFAST_PCD_DIR` 指向联想本地目录。

## 交付判断

完整状态机模拟通过、Linux 宿主机运行通过、UI 导入/本机 IPC 通过，与真实机械全部动作和 USB-RS485 稳定性通过，是不同验收项。

先完成本机机构完整流程与单机底盘安全验收，再接实际定位/导航及雷达负载，做并行长稳与断链测试。任何停止未确认、误接串口、位置不一致、自动续跑或未处理的 USB 重枚举，都不应判定全迁可交付。本次保留原双机部署作为回退，不改动正在运行的雷达任务。

## 本次验证记录（2026-10-10）

- 本地共享驱动、原状态机/编排、机构、串口、安全逻辑和新联想代码回归：`362 passed, 1 skipped`；跳过项为 Windows 上不可运行的 Linux Socket 测试，该项已在联想执行通过。
- 联想 Python 3.10 环境：`57 passed`，含实际 Unix Socket 权限、请求、停止和清理验证；硬件后端均为模拟器。
- 联想完整原流程模拟：两点、每点浅/中/深三仓，完成加管、下压、回程、输粮、取管和返航状态；共 112 次 **模拟** 伺服触发，最终 `COMPLETED`。
- 联想 Qt 6.12.0 导入、单页面渲染、真实 MainWindow + 模拟守护进程端到端检查通过。状态栏区分“模拟硬件”和“本机直连”，机构阶段标示“人工到位”，不误称自动导航已运行。
- 证据位于部署目录：`validation-final-pytest.log`、`validation/full-flow/report.json`、`validation-ui-stack.log`、`validation/ui-stack/main-window.png`。日志和截图只证明软件验证范围，不证明实际串口接线、真实位置或机械停止。
- 早期一次本地回归遇到原共享串口并发序号测试的时序失败；单项与后续完整回归通过。未为了追求序号顺序改变停止命令的抢占行为，真实并发负载仍需实机验收。

尚未验收：两条实际 USB 链路、固件全部动作/档位/急停一致性、连续 RS485 往复及 USB 重枚举、雷达负载下底盘与机构并行、实际地图/定位质量信号与导航。当前交付的是可在联想独立运行的机构完整流程验证版本，不是已完成实机验收的全自主导航生产版本。

## 首次联想实机启动记录（2026-10-10）

- 现场确认后，以单独的 `LENOVO/config/local_robot.operator-live.json` 尝试 `operator` 模式启动。STM32 串口可打开，但启动时 3 秒内无有效状态帧；独立只接收诊断在 115200 波特率下 4 秒收到 0 字节。硬件服务安全退出，没有建立机构任务。
- X2P USB-RS485 以 9600 波特率读取从站 2 的编码器和速度寄存器成功，读数为 0；尚未发送运动命令。需等 STM32 供电、接线和状态回传确认后，再重新启动服务。
- 联想 Qt 6 的 xcb 平台插件需要 `libxcb-cursor0`。本次把 Ubuntu 包下载并解压到部署目录 `.local-qt-deps/extracted/`，只给 UI 进程设置 `LD_LIBRARY_PATH`，没有改动系统 Python 或全局软件包。若在图形桌面终端手动重启 UI，可执行：

```bash
cd /home/ps02/grain_robot/staging/lenovo-local-20261010
LD_LIBRARY_PATH="$PWD/.local-qt-deps/extracted/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .venv/bin/python LENOVO/scripts/local_robot.py ui \
  --config LENOVO/config/local_robot.operator-live.json
```

UI 现在允许在本机硬件服务未启动时显示离线状态；选择任务时会重新核验本机服务身份和导航模式，服务未就绪不会创建机构任务。图形桌面上的 `grain-lenovo-operator-ui2.service` 为本次临时 UI 单元，不是开机自启动。实机硬件服务已退出，也没有自动重试。

## 联想人工到位联调一键启动

在联想图形桌面终端，从发布包根目录运行：

```bash
bash LENOVO/scripts/start_operator_bench.sh
```

脚本会交互确认现场有人监护、行程清空、硬件急停/防坠可用，以及吸粮风机由现行固件或现场外部处理。需要非交互运行时，操作者可在本次会话明确传入 `--attended --external-suction-confirmed`；这两个参数不代表完成了硬件验收。默认读取 `LENOVO/config/local_robot.operator-live.json`，也可用 `--config /绝对路径/live.json` 指定。配置必须为 `operator` 人工到位模式，升降速度从配置读取（当前 30 rpm），不会继承旧 3588 联调脚本的 800 rpm。

脚本检查两条稳定 USB 身份、串口占用和现有本机服务；**发现已有服务时拒绝启动，不会自动关闭它**。目前临时的 `grain-lenovo-operator2.service` 和 `grain-lenovo-operator-ui2.service` 若仍在运行，先在现场确认当前任务/机构已安全停止，再由操作者结束原会话，之后才可使用此脚本。不可两套并行抢串口。

此入口只启动本机直连守护进程和 UI，不启动雷达、自主导航、工单或任何机构动作。UI 关闭时脚本向自己启动的服务发送停止请求并结束该服务；如果停止未获确认，必须现场检查并使用硬件急停。日志保存到配置运行目录的 `bench/` 子目录，不自动删除未完成工单或机械复位记录。若通过 SSH 启动 UI，应显式配置 `DISPLAY` 和需要的 `XAUTHORITY`；无图形显示会拒绝启动。

2026-10-10 20:24，修正脚本的包搜索顺序，并把检查脚本的 `PYTHONPATH` 与 UI 进程隔离后，在联想启动 `grain-lenovo-bench4.service` 成功。实机本机 IPC 返回 `runtime=lenovo-local`、`simulation=false`、STM32 在线、RC 自动档有效、故障 0、底盘未授权运动；UI 进程及可见的“粮食扦样机器人 — 联想本机直连”窗口均已确认。此记录只验收启动、连通和状态读取，**未创建任务，也未验证机构动作、急停实际停机效果或长时间 RS485 稳定性**。本次日志位于 `/home/ps02/.local/state/grain-lenovo/bench/20261010-202452-263682/`。结束本次会话应优先正常关闭 UI；异常时先使用硬件急停，再检查服务和串口状态。

2026-10-10 20:26 的 Ubuntu 崩溃提示来自 UI Python 进程（退出码 134），不是启动脚本或 STM32 驱动报错。堆栈位于 PySide6 跨线程信号回调和按钮样式刷新；原实现把 100 ms 状态轮询同时用于完整状态字典的 10 Hz Qt 信号和重复样式重绘。已去除遥控档位信号的重复连接、仅在显示状态变化时重绘，并对本机状态信号做变化去重，保留 100 ms UI/服务心跳。联想端相关回归 `35 passed`；修复后 `grain-lenovo-bench6.service` 的 UI 在实机运行约 2 分 47 秒未再出现 Qt 崩溃，随后因下述机械安全问题主动结束。**这不是长稳或机械验收**。旧的 Apport “Ubuntu” 弹窗和 `/var/crash/_usr_bin_python3.10.1000.crash` 来自此前崩溃，未删除。

本次 UI 运行期间现场有人从界面触发了机构流程：第一次升降中因“fresh RC automatic mode required”锁停，之后人工复位并再次尝试；第二次夹紧回包超时，人工复位后再次升降，仍因遥控自动档新鲜度不足锁停。服务最后检查到 `requires_mechanical_reset=true`，`lift-recovery.json` 保持 `recovery_pending=true`。已停止 `grain-lenovo-bench6.service`，两个串口无进程占用。**不要删除恢复文件、再次复位或重新启动实机任务**；现场先确认夹爪、取样管和升降实际位置/静止及硬件急停，再单独排查 STM32 命令回包和遥控档位波动。`stop_submitted` 事件仅表示停止指令写出，不表示机构实际停机得到反馈确认。
