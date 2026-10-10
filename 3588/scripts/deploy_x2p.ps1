# 板端部署脚本 — X2P 伺服升降 + 停止断电释放 + 遥控新标定 + UI 启动优化
# 板子：Orange Pi 5 Max @ 192.168.1.114（orangepi/orangepi）
# 用法（板子在线时，PowerShell）：
#   powershell -ExecutionPolicy Bypass -File scripts/deploy_x2p.ps1
#
# 设计：所有板端复杂命令都写成 /tmp/deploy_x2p.sh 上传后 bash 执行，
# 避免 PowerShell -> plink 内联转义问题。

$ErrorActionPreference = "Stop"
$HOSTKEY = "SHA256:zdmfi6OsyVhMiOdLDKzcgx28QgPA7c7fvFD9GLIAp/0"
$BOARD = "192.168.1.114"
$USER = "orangepi"
$PASS = "orangepi"
$PSCP = "C:\Program Files\PuTTY\pscp.exe"
$PLINK = "C:\Program Files\PuTTY\plink.exe"
$LOCAL = "E:\青赋驭境\项目\粮食扦样\grain_sampling_robot_software"
$REMOTE = "/home/orangepi/grain_sampling_robot_software"

Write-Host "=== 1. 测试连接 ===" -ForegroundColor Cyan
$online = Test-Connection -ComputerName $BOARD -Count 1 -Quiet
if (-not $online) {
    Write-Host "[FAIL] 板子离线，无法部署" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] 板子在线"

Write-Host "=== 2. 同步源码文件 ===" -ForegroundColor Cyan
$srcFiles = @(
    "src/grain_sampling_devices/mechanism_driver.py",
    "src/grain_sampling_devices/x2p_lift.py",
    "src/grain_sampling_workflow/mechanism_node.py",
    "src/grain_sampling_workflow/rc_control.py",
    "src/grain_sampling_ui/main.py",
    "src/grain_sampling_workflow/slam_bridge.py"
)
foreach ($f in $srcFiles) {
    $localPath = $LOCAL + "\" + $f
    $remotePath = $USER + "@" + $BOARD + ":" + $REMOTE + "/" + $f
    & $PSCP -batch -hostkey $HOSTKEY -pw $PASS $localPath $remotePath
    Write-Host ("  " + $f)
}

Write-Host "=== 3. 拷贝 x2p 包 ===" -ForegroundColor Cyan
& $PSCP -batch -hostkey $HOSTKEY -pw $PASS -r ($LOCAL + "\src\x2p") ($USER + "@" + $BOARD + ":" + $REMOTE + "/src/")

Write-Host "=== 4. 上传板端部署脚本并执行 ===" -ForegroundColor Cyan
& $PSCP -batch -hostkey $HOSTKEY -pw $PASS ($LOCAL + "\scripts\deploy_x2p_board.sh") ($USER + "@" + $BOARD + ":" + $REMOTE + "/scripts/deploy_x2p_board.sh")
& $PLINK -ssh -batch -hostkey $HOSTKEY -l $USER -pw $PASS $BOARD ("bash " + $REMOTE + "/scripts/deploy_x2p_board.sh")

Write-Host ""
Write-Host "=== 部署完成 ===" -ForegroundColor Green
Write-Host "实机验证：rosservice call /mechanism/press '{}'   /mechanism/lift '{}'"
Write-Host "[WARN] 垂直轴需硬件急停+上下限位+防坠装置"
