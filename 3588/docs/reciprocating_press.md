# 正式流程与插管测试共用往复下压

UI 正式作业与 `scripts/sampling_press.py` 均调用
`/mechanism/move_lift(direction="down_cycle")`，往复算法仅在
`src/grain_sampling_devices/mechanism_driver.py` 实现。
假导航脚本只跳过导航，不替换机构动作。

在根目录 `sampling_params.py` 修改参数后，需重启机构服务和 UI：

- `PRESS_SEGMENT_CM = 20.0`：每段净下压目标，单位 cm。
- `PRESS_DOWN_CM = 5.0`：每次向下距离，单位 cm。
- `PRESS_UP_CM = 2.0`：中间回拉距离，单位 cm，必须小于向下距离。
- `PRESS_PAUSE_S = 0.5`：动作之间的额外停顿，单位秒。
- 夹紧、松开时长仍使用既有品种参数，不使用旧脚本的硬编码脉宽和时间。

默认段内目标深度依次为 5、3、8、6、11、9……20 cm。
末程在目标深度停止，不再回拉，也不发出超过段终点的目标。
每个目标按本段编码器原点计算，不累计相对运动误差。
这限制的是命令目标；实际机械惯性、限位和制动性能仍须实机验证。

每段动作：夹紧 → 往复下压 → 松开 → 绝对回程 → 夹紧。
回程直接以本轮开始时保存的编码器绝对位置为目标，
不再保留原点下方余量，避免多轮送管后累计向下偏移。
运动报错后保留原点，不自动回程、不自动重试。
急停会请求停止伺服，并阻止后续往复动作；不能替代硬件急停。

三终端 UI 测试仍按原流程操作，不要同时运行独立插管脚本。
只测试插管时，可在机构已安全就位、UI 作业停止后执行：

```bash
python3 scripts/sampling_press.py 20
```

该命令需要输入 YES 才运动。旧的 down_cm/up_cm 命令行参数已取消，
避免客户端与正式服务参数不一致。脚本最后一段也执行松开、回程、夹紧，
与正式流程一致；它不执行后续吸粮和输送步骤。

机构日志中搜索 `LIFT_CYCLE_ORIGIN_SAVED`、`LIFT_CYCLE_LEG`、
`LIFT_CYCLE_COMPLETE`、`LIFT_CYCLE_FAILED`、`LIFT_CYCLE_RETURN`。
部署前确认没有作业运行，不要在机构悬停或故障状态下直接重启继续测试。
