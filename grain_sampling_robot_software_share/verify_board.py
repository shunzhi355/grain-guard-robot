import paramiko
import json

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.1.111', username='orangepi', password='orangepi', timeout=10)

def run(cmd):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=10)
    return stdout.read().decode().strip()

results = {}

# 1. Deployed code
results['attach_orch'] = run("grep -c 'def attach_orchestrator' ~/grain_sampling_project/grain_sampling_software/src/grain_sampling_ui/pages/guidance_page.py")
results['get_task_by_id'] = run("grep -c 'def get_task_by_id' ~/grain_sampling_project/grain_sampling_software/src/grain_sampling_ui/pages/task_list_page.py")
results['start_sampling'] = run("grep 'start_sampling.*task_list' ~/grain_sampling_project/grain_sampling_software/src/grain_sampling_ui/main.py")

# 2. UI process
results['ui_running'] = run("ps aux | grep grain_sampling_ui | grep -v grep | wc -l")
results['ui_log'] = run("tail -5 /tmp/ui_output.log 2>/dev/null || echo 'no log'")

# 3. SLAM
results['pcd_count'] = run("ls ~/fastlio2_ws/src/S-FAST_LIO/PCD/ 2>/dev/null | wc -l")

# 4. Point cloud
results['pc_src'] = run("ls ~/grain_sampling_project/grain_sampling_software/src/grain_sampling_pointcloud/ 2>/dev/null || echo 'not found'")

# 5. Files on board
results['checklist'] = run("grep -c '不通过' ~/grain_sampling_project/grain_sampling_software/scripts/acceptance_checklist.md")
results['systemd_service'] = run("wc -c < ~/grain_sampling_project/grain_sampling_software/scripts/grain-sampling.service 2>/dev/null || echo 0")

ssh.close()

for k, v in sorted(results.items()):
    print(f"  {k}: {v}")
