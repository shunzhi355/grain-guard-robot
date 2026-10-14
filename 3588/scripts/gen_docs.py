#!/usr/bin/env python3
"""Generate ROS interface alignment table and client interface document as .docx files."""

import os
from docx import Document
from docx.shared import Pt, Inches, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn


def set_cell_shading(cell, color):
    """Set cell background color."""
    shading = cell._element.get_or_add_tcPr()
    shd_elem = shading.makeelement(qn('w:shd'), {
        qn('w:fill'): color,
        qn('w:val'): 'clear',
    })
    shading.append(shd_elem)


def add_table(doc, headers, rows, col_widths=None):
    """Add a formatted table to the document."""
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    table.style = 'Table Grid'
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    # Header row
    hdr = table.rows[0]
    for i, header in enumerate(headers):
        cell = hdr.cells[i]
        cell.text = ''
        p = cell.paragraphs[0]
        run = p.add_run(header)
        run.bold = True
        run.font.size = Pt(10)
        run.font.name = '微软雅黑'
        run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        set_cell_shading(cell, 'D9E2F3')

    # Data rows
    for r, row_data in enumerate(rows):
        row = table.rows[r + 1]
        for c, cell_text in enumerate(row_data):
            cell = row.cells[c]
            cell.text = ''
            p = cell.paragraphs[0]
            run = p.add_run(str(cell_text))
            run.font.size = Pt(9)
            run.font.name = '微软雅黑'
            run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')
            p.alignment = WD_ALIGN_PARAGRAPH.LEFT

    # Apply column widths if provided
    if col_widths:
        for row in table.rows:
            for i, width in enumerate(col_widths):
                row.cells[i].width = Cm(width)

    doc.add_paragraph('')
    return table


def add_heading(doc, text, level=2):
    """Add a heading with Chinese font."""
    heading = doc.add_heading(text, level=level)
    for run in heading.runs:
        run.font.name = '微软雅黑'
        run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')
    return heading


def add_para(doc, text, bold=False, size=10, alignment=None):
    """Add a paragraph with Chinese font."""
    p = doc.add_paragraph()
    run = p.add_run(text)
    run.font.size = Pt(size)
    run.font.name = '微软雅黑'
    run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')
    run.bold = bold
    if alignment is not None:
        p.alignment = alignment
    return p


def add_code_block(doc, code_text):
    """Add a code block with monospace font and light background."""
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.5)
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after = Pt(6)
    run = p.add_run(code_text)
    run.font.size = Pt(9)
    run.font.name = 'Consolas'
    run.font.color.rgb = RGBColor(0x1A, 0x1A, 0x1A)
    # Add shading
    pPr = p._element.get_or_add_pPr()
    shd = pPr.makeelement(qn('w:shd'), {
        qn('w:fill'): 'F2F2F2',
        qn('w:val'): 'clear',
    })
    pPr.append(shd)
    return p


def add_checkbox(doc, text="确认 ☐"):
    """Add a checkbox line."""
    p = doc.add_paragraph()
    run = p.add_run(text)
    run.font.size = Pt(10)
    run.font.name = '微软雅黑'
    run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')
    p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    return p


def generate_ros_alignment_doc(output_path):
    """Generate ROS 接口对齐表.docx"""
    doc = Document()

    # Set default font
    style = doc.styles['Normal']
    font = style.font
    font.name = '微软雅黑'
    font.size = Pt(10)
    style.element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')

    # Title
    title = doc.add_heading('ROS 接口对齐表 —— 粮食扦样机器人', level=0)
    for run in title.runs:
        run.font.name = '微软雅黑'
        run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')

    # Header note
    add_para(doc, '本文档用于各团队（底盘/导航/SLAM/机构/UI）对齐 ROS 接口。请逐项确认 Topic/Service 名称、类型、提供方。',
             size=10)

    # ===== Section 1: 订阅的 Topic =====
    add_heading(doc, '1. 我方（UI团队）订阅的 Topic', level=1)

    topic_headers = ['Topic', '消息类型', '频率', '提供方', '用途']
    topic_rows = [
        ['/odometry/filtered', 'nav_msgs/Odometry', '~50Hz', '导航团队(EKF)', 'UI显示机器人位置'],
        ['/map', 'nav_msgs/OccupancyGrid', '0.1~1Hz', 'SLAM团队', 'UI显示仓库栅格地图'],
        ['/tf', 'tf2_msgs/TFMessage', '~50Hz', '导航团队', '坐标变换'],
        ['/livox/lidar', 'sensor_msgs/PointCloud2', '10Hz', 'LiDAR驱动', '点云采集与切面提取'],
        ['/camera/image_raw', 'sensor_msgs/Image', '15fps', '摄像头驱动', '实时视频流'],
        ['/mechanism/status', 'std_msgs/String(JSON)', '1~5Hz', '机构团队', '机构状态(深度/负压/重量)'],
    ]
    add_table(doc, topic_headers, topic_rows, col_widths=[3.5, 3.0, 1.5, 2.5, 4.5])
    add_checkbox(doc)

    # ===== Section 2: 调用的 Service =====
    add_heading(doc, '2. 我方（UI团队）调用的 Service', level=1)

    service_headers = ['Service', '类型', '说明', '提供方']
    service_rows = [
        ['/navigate_to_pose', 'nav2_msgs/NavigateToPose', '发送导航目标点(X,Y)', '导航团队'],
        ['/mechanism/start_sampling', 'std_srvs/Trigger', '启动扦样', '机构团队'],
        ['/mechanism/stop_sampling', 'std_srvs/Trigger', '停止扦样', '机构团队'],
        ['/mechanism/pause_sampling', 'std_srvs/Trigger', '暂停吸粮', '机构团队'],
        ['/mechanism/resume_sampling', 'std_srvs/Trigger', '恢复吸粮', '机构团队'],
        ['/slam/start_mapping', 'std_srvs/Trigger', '开始建图', 'SLAM团队'],
        ['/slam/stop_mapping', 'std_srvs/Trigger', '停止建图', 'SLAM团队'],
        ['/slam/save_map', 'std_srvs/Trigger', '保存地图', 'SLAM团队'],
        ['/emergency_stop', 'std_srvs/Trigger', '全系统急停', '全系统'],
    ]
    add_table(doc, service_headers, service_rows, col_widths=[4.5, 3.5, 4.0, 2.5])
    add_checkbox(doc)

    # ===== Section 3: 坐标系约定 =====
    add_heading(doc, '3. 坐标系约定', level=1)

    tf_headers = ['Frame', '父 Frame', '说明']
    tf_rows = [
        ['map', '—', '世界坐标系原点'],
        ['odom', 'map', '里程计坐标系'],
        ['base_link', 'odom', '机器人本体中心'],
        ['laser', 'base_link', 'Livox Mid-360 激光雷达'],
        ['camera', 'base_link', 'USB 摄像头'],
    ]
    add_table(doc, tf_headers, tf_rows, col_widths=[3.5, 3.5, 7.0])
    add_checkbox(doc)

    # ===== Section 4: /mechanism/status JSON格式约定 =====
    add_heading(doc, '4. /mechanism/status JSON格式约定', level=1)

    json_text = '''{
  "state": "idle",
  "depth": 1.5,
  "pressure": -3.2,
  "bin_weights": [2.3, 0, 1.8],
  "pipe_index": 3,
  "error_code": 0
}'''
    add_code_block(doc, json_text)

    add_para(doc, '字段说明：', bold=True, size=9)
    field_descriptions = [
        'state: idle | pressing | suctioning | conveying | error',
        'depth: 当前扦样深度（米）',
        'pressure: 负压吸粮系统压力值（kPa）',
        'bin_weights: 三个分仓各自重量（kg），顺序[1号仓, 2号仓, 3号仓]',
        'pipe_index: 当前已连接管节序号',
        'error_code: 0=正常，非0=故障码',
    ]
    for desc in field_descriptions:
        add_para(doc, f'  {desc}', size=9)

    add_checkbox(doc)

    # ===== Section 5: 确认回执 =====
    add_heading(doc, '5. 确认回执', level=1)

    receipt_headers = ['团队', '确认人', '日期', '修改意见']
    receipt_rows = [
        ['底盘驱动', '', '', ''],
        ['导航/SLAM', '', '', ''],
        ['机构控制', '', '', ''],
        ['UI/操作界面', '', '', ''],
    ]
    add_table(doc, receipt_headers, receipt_rows, col_widths=[3.5, 3.5, 3.5, 4.5])

    doc.save(output_path)
    print(f'Generated: {output_path}')


def generate_client_interface_doc(output_path):
    """Generate 甲方接口文档.docx"""
    doc = Document()

    # Set default font
    style = doc.styles['Normal']
    font = style.font
    font.name = '微软雅黑'
    font.size = Pt(10)
    style.element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')

    # Title
    title = doc.add_heading('云端及设备接口文档 —— 粮食扦样机器人', level=0)
    for run in title.runs:
        run.font.name = '微软雅黑'
        run._element.rPr.rFonts.set(qn('w:eastAsia'), '微软雅黑')

    # Header note
    add_para(doc, '本文档用于与甲方（云端系统、生化/理化设备供应商）对齐接口协议。', size=10)

    # ===== Section 1: 云端通信协议 =====
    add_heading(doc, '1. 云端通信协议（MQTT + JSON）', level=1)

    mqtt_headers = ['项目', '内容']
    mqtt_rows = [
        ['协议', 'MQTT v5 / paho-mqtt'],
        ['Broker', '甲方提供地址'],
        ['Topic前缀', 'robot/{device_id}/'],
        ['认证方式', 'Username + Password（甲方提供）'],
        ['QoS', '默认1（可配置）'],
        ['消息格式', 'JSON（UTF-8编码）'],
        ['TLS/SSL', '甲方确认是否需要'],
    ]
    add_table(doc, mqtt_headers, mqtt_rows, col_widths=[4.0, 10.0])

    # ===== Section 2: MQTT Topic 规划 =====
    add_heading(doc, '2. MQTT Topic 规划', level=1)

    mqtt_topic_headers = ['Topic', '方向', '说明']
    mqtt_topic_rows = [
        ['robot/{id}/work_order_request', '机器人→云', '请求工单列表'],
        ['robot/{id}/work_order_response', '云→机器人', '返回工单列表'],
        ['robot/{id}/task_status', '机器人→云', '上报任务状态（started/completed/error）'],
        ['robot/{id}/map_upload', '机器人→云', '上传地图数据'],
        ['robot/{id}/telemetry', '机器人→云', '实时遥测数据'],
        ['robot/{id}/command', '云→机器人', '云端下发指令'],
    ]
    add_table(doc, mqtt_topic_headers, mqtt_topic_rows, col_widths=[5.5, 3.0, 5.5])

    # ===== Section 3: 工单请求/响应格式 =====
    add_heading(doc, '3. 工单请求/响应格式', level=1)

    add_para(doc, '工单请求（机器人→云）：', bold=True, size=10)
    req_json = '''{
  "message_type": "work_order_request",
  "warehouse_id": "WH-001",
  "timestamp": "2026-07-15T10:30:00"
}'''
    add_code_block(doc, req_json)

    add_para(doc, '工单响应（云→机器人）：', bold=True, size=10)
    resp_json = '''{
  "message_type": "work_order_response",
  "orders": [
    {
      "order_id": "WO-20260715001",
      "warehouse": "1号廒间",
      "depth_list": [0.5, 1.0, 1.5],
      "points": [{"x": 10.0, "y": 20.0}, {"x": 15.0, "y": 25.0}],
      "status": "pending"
    }
  ]
}'''
    add_code_block(doc, resp_json)

    # ===== Section 4: 任务状态上报格式 =====
    add_heading(doc, '4. 任务状态上报格式', level=1)

    status_json = '''{
  "message_type": "task_status_report",
  "order_id": "WO-20260715001",
  "status": "started",
  "timestamp": "2026-07-15T10:35:00"
}'''
    add_code_block(doc, status_json)

    add_para(doc, 'status值: started | completed | error', size=10)

    # ===== Section 5: 生化/理化设备接口需求 =====
    add_heading(doc, '5. 生化/理化设备接口需求', level=1)

    device_headers = ['设备', '协议', '默认端口', '说明']
    device_rows = [
        ['生化分析仪', 'TCP/JSON', '5001', '测试水分、脂肪酸值等'],
        ['理化分析仪', 'TCP/JSON', '5002', '测试容重、不完善粒等'],
    ]
    add_table(doc, device_headers, device_rows, col_widths=[3.0, 2.5, 2.5, 6.0])

    add_para(doc, '接口需求（请甲方/设备供应商提供）：', bold=True, size=10)
    requirements = [
        'TCP连接的握手协议格式',
        '命令帧格式（发送什么JSON/二进制数据）',
        '响应帧格式（返回什么数据）',
        '心跳/超时机制',
        '错误码定义',
    ]
    for i, req in enumerate(requirements, 1):
        add_para(doc, f'{i}. {req}', size=10)

    # ===== Section 6: 视频流（RTSP）需求 =====
    add_heading(doc, '6. 视频流（RTSP）需求', level=1)

    rtsp_headers = ['项目', '内容']
    rtsp_rows = [
        ['协议', 'RTSP over TCP'],
        ['视频编码', 'H.264 或 MJPEG'],
        ['分辨率', '640x480 或 1280x720'],
        ['帧率', '15-30 fps'],
        ['RTSP地址', '甲方提供（如 rtsp://192.168.1.x:554/stream）'],
    ]
    add_table(doc, rtsp_headers, rtsp_rows, col_widths=[4.0, 10.0])

    # ===== Section 7: 待甲方确认事项 =====
    add_heading(doc, '7. 待甲方确认事项', level=1)

    confirm_headers = ['序号', '事项', '状态', '回复']
    confirm_rows = [
        ['1', 'MQTT Broker地址', '待确认', ''],
        ['2', 'MQTT认证方式（用户名/密码）', '待确认', ''],
        ['3', '是否需要TLS/SSL', '待确认', ''],
        ['4', '工单JSON格式是否匹配', '待确认', ''],
        ['5', '生化分析仪API文档', '待确认', ''],
        ['6', '理化分析仪API文档', '待确认', ''],
        ['7', 'RTSP视频流地址', '待确认', ''],
        ['8', '点云切面数据上传格式', '待确认', ''],
    ]
    add_table(doc, confirm_headers, confirm_rows, col_widths=[1.5, 5.5, 2.5, 5.0])

    doc.save(output_path)
    print(f'Generated: {output_path}')


def main():
    """Generate both documents."""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    docs_dir = os.path.join(base_dir, 'docs')
    os.makedirs(docs_dir, exist_ok=True)

    ros_path = os.path.join(docs_dir, 'ROS接口对齐表.docx')
    client_path = os.path.join(docs_dir, '甲方接口文档.docx')

    generate_ros_alignment_doc(ros_path)
    generate_client_interface_doc(client_path)

    print('Done. Both documents generated successfully.')


if __name__ == '__main__':
    main()
