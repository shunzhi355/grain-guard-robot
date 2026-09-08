"""
Update 甲方接口文档.docx from MQTT spec to HTTP API spec.

Run: python scripts/update_client_doc_v2.py

Modifies:
  Section 1: heading "MQTT" -> "HTTP", replace protocol table
  Section 2: heading "MQTT Topic" -> "HTTP API", replace topics table
  Section 3: update JSON examples (remove message_type, rename fields, add aojian/jiance)
  Section 4: update status report JSON (remove message_type, add paused/abandoned)
  Section 8: replace all confirmation items to HTTP context

Preserves sections 5 (生化/理化), 6 (RTSP), 7 (点云轮廓) untouched.
"""

import docx
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

DOC_PATH = r"E:\青赋驭境\项目\粮食扦样\grain_sampling_robot_software\docs\甲方接口文档.docx"


# ── helpers ──────────────────────────────────────────────────────────

def clear_paragraph_runs(p):
    """Remove all <w:r> children from a paragraph element."""
    p_el = p._element
    for child in list(p_el):
        if child.tag == qn('w:r'):
            p_el.remove(child)


def set_paragraph_text(p, text, font_name=None, font_size=None):
    """
    Replace paragraph content with given text.
    For multi-line text, uses <w:br/> between lines within a single run.
    font_name: 'Consolas' for JSON, None for default.
    """
    clear_paragraph_runs(p)
    lines = text.split('\n')
    r = OxmlElement('w:r')
    # Font properties
    if font_name or font_size:
        rPr = OxmlElement('w:rPr')
        if font_name:
            rFonts = OxmlElement('w:rFonts')
            rFonts.set(qn('w:ascii'), font_name)
            rFonts.set(qn('w:hAnsi'), font_name)
            rFonts.set(qn('w:eastAsia'), font_name)
            rPr.append(rFonts)
        if font_size:
            sz = OxmlElement('w:sz')
            sz.set(qn('w:val'), str(int(font_size * 2)))  # half-points
            rPr.append(sz)
        r.append(rPr)
    # Text with line breaks
    for i, line in enumerate(lines):
        t = OxmlElement('w:t')
        t.set(qn('xml:space'), 'preserve')
        t.text = line
        r.append(t)
        if i < len(lines) - 1:
            r.append(OxmlElement('w:br'))
    p._element.append(r)


def replace_table_contents(table, rows_data, col_widths=None):
    """
    Remove all existing rows from table, add new rows.
    rows_data[0] is header row.
    col_widths: list of column widths in dxa (optional).
    """
    tbl_el = table._element
    # Remove existing rows
    for row_el in list(tbl_el.findall(qn('w:tr'))):
        tbl_el.remove(row_el)
    # Add new rows
    for ri, row_cells in enumerate(rows_data):
        tr = OxmlElement('w:tr')
        for ci, cell_text in enumerate(row_cells):
            tc = OxmlElement('w:tc')
            if col_widths and ci < len(col_widths):
                tcW = OxmlElement('w:tcW')
                tcW.set(qn('w:w'), str(col_widths[ci]))
                tcW.set(qn('w:type'), 'dxa')
                tc.append(tcW)
            p_el = OxmlElement('w:p')
            r_el = OxmlElement('w:r')
            if ri == 0:
                rPr = OxmlElement('w:rPr')
                rPr.append(OxmlElement('w:b'))
                r_el.append(rPr)
            t_el = OxmlElement('w:t')
            t_el.set(qn('xml:space'), 'preserve')
            t_el.text = str(cell_text)
            r_el.append(t_el)
            p_el.append(r_el)
            tc.append(p_el)
            tr.append(tc)
        tbl_el.append(tr)


def find_paragraph_containing(doc, text_fragment, style_name=None):
    """Find first paragraph whose text contains the fragment."""
    for p in doc.paragraphs:
        if text_fragment in p.text:
            if style_name is None or p.style.name == style_name:
                return p
    return None


def find_table_by_first_cell(tables, cell_text):
    """Find table whose first cell text matches."""
    for t in tables:
        if t.rows and len(t.rows[0].cells) > 0:
            if t.rows[0].cells[0].text.strip() == cell_text:
                return t
    return None


# ── main ─────────────────────────────────────────────────────────────

def main():
    doc = docx.Document(DOC_PATH)
    warnings = []

    # ================================================================
    # STEP 1: Section 1 heading + protocol table (Table 0)
    # ================================================================
    h1 = find_paragraph_containing(doc, "云端通信协议", "Heading 1")
    if h1:
        set_paragraph_text(h1, "1. 云端通信协议（HTTP + JSON）")
        print("  [OK] S1 heading: MQTT -> HTTP")
    else:
        warnings.append("S1 heading '云端通信协议' not found")

    tbl0 = find_table_by_first_cell(doc.tables, "项目")
    if tbl0 and "MQTT" in (tbl0.rows[1].cells[1].text if len(tbl0.rows) > 1 else ""):
        replace_table_contents(tbl0, [
            ["项目", "内容"],
            ["协议", "HTTP/1.1 REST API"],
            ["Base URL", "甲方提供"],
            ["认证方式", "每请求携带设备MAC地址（JSON body中）"],
            ["请求格式", "POST JSON (Content-Type: application/json)"],
            ["响应格式", "JSON"],
            ["消息格式", "UTF-8编码"],
            ["重试策略", "网络超时自动重试（最多3次，指数退避）"],
        ], col_widths=[2400, 6600])
        print("  [OK] Table 0: MQTT -> HTTP protocol")
    else:
        warnings.append("Table 0 (MQTT protocol) not found or already updated")

    # ================================================================
    # STEP 2: Section 2 heading + API table (Table 1)
    # ================================================================
    h2 = find_paragraph_containing(doc, "MQTT Topic", "Heading 1")
    if h2:
        set_paragraph_text(h2, "2. HTTP API 接口清单")
        print("  [OK] S2 heading: MQTT Topic -> HTTP API")
    else:
        warnings.append("S2 heading 'MQTT Topic' not found")

    tbl1 = find_table_by_first_cell(doc.tables, "Topic")
    if tbl1:
        replace_table_contents(tbl1, [
            ["接口", "方法", "路径", "说明"],
            ["任务列表", "POST", "/api/tasks", "上传MAC -> 返回粮库+任务"],
            ["承接任务", "POST", "/api/tasks/accept", "承接指定任务"],
            ["状态上报", "POST", "/api/tasks/status", "完成/暂停/放弃"],
            ["间列表", "POST", "/api/rooms", "获取粮库下间列表"],
            ["地图上传", "POST", "/api/maps", "上传切面轮廓坐标"],
        ], col_widths=[1800, 900, 2200, 4100])
        print("  [OK] Table 1: MQTT topics -> HTTP API list")
    else:
        warnings.append("Table 1 (MQTT Topics) not found")

    # ================================================================
    # STEP 3: Section 3 -- 工单请求/响应 JSON
    # ================================================================
    # P9: description text (use specific substring to avoid matching heading P8)
    p9 = find_paragraph_containing(doc, "工单请求（机器人")
    if p9:
        set_paragraph_text(p9, "请求（POST /api/tasks）：")
        print("  [OK] P9: request label updated")
    else:
        warnings.append("P9 '工单请求（机器人' not found")

    # P10: request JSON (old: message_type=work_order_request)
    p10 = find_paragraph_containing(doc, '"work_order_request"')
    if p10:
        set_paragraph_text(p10,
            '{\n'
            '  "mac": "00:1A:2B:3C:4D:5E"\n'
            '}',
            font_name='Consolas', font_size=9)
        print("  [OK] P10: request JSON -> mac only")
    else:
        warnings.append("P10 'work_order_request' JSON not found")

    # P11: response description (use specific substring to avoid matching heading P8)
    p11 = find_paragraph_containing(doc, "工单响应（云")
    if p11:
        set_paragraph_text(p11, "响应（POST /api/tasks）：")
        print("  [OK] P11: response label updated")
    else:
        warnings.append("P11 '工单响应（云' not found")

    # P12: response JSON (old: message_type=work_order_response)
    p12 = find_paragraph_containing(doc, '"work_order_response"')
    if p12:
        set_paragraph_text(p12,
            '{\n'
            '  "liangku_id": "LK-001",\n'
            '  "liangku": "1号粮库",\n'
            '  "orders": [\n'
            '    {\n'
            '      "order_id": "WO-20260715001",\n'
            '      "aojian_id": "AJ-001",\n'
            '      "aojian": "1号廒间",\n'
            '      "depth_list": [0.5, 1.0, 1.5],\n'
            '      "jiance": [1, 2, 3],\n'
            '      "points": [{"x": 10.0, "y": 20.0}, {"x": 15.0, "y": 25.0}]\n'
            '    }\n'
            '  ]\n'
            '}',
            font_name='Consolas', font_size=9)
        print("  [OK] P12: response JSON -> liangku_id, aojian, jiance")
    else:
        warnings.append("P12 'work_order_response' JSON not found")

    # ================================================================
    # STEP 4: Section 4 -- 任务状态上报 JSON
    # ================================================================
    p14 = find_paragraph_containing(doc, '"task_status_report"')
    if p14:
        set_paragraph_text(p14,
            '{\n'
            '  "mac": "00:1A:2B:3C:4D:5E",\n'
            '  "order_id": "WO-20260715001",\n'
            '  "status": "started",\n'
            '  "timestamp": "2026-07-15T10:35:00",\n'
            '  "results": {\n'
            '    "water_content": 13.5,\n'
            '    "bulk_density": 780\n'
            '  }\n'
            '}',
            font_name='Consolas', font_size=9)
        print("  [OK] P14: status report JSON -> HTTP format")
    else:
        warnings.append("P14 'task_status_report' JSON not found")

    p15 = find_paragraph_containing(doc, "status值")
    if p15:
        set_paragraph_text(p15, "status值: started | completed | paused | abandoned（完成后附带 results）")
        print("  [OK] P15: status values updated")
    else:
        warnings.append("P15 'status值' not found")

    # ================================================================
    # STEP 5: Section 8 -- 待甲方确认事项 table (Table 5)
    # ================================================================
    tbl5 = find_table_by_first_cell(doc.tables, "序号")
    # Verify it's the confirmation table (first data row has 待确认)
    if tbl5 and len(tbl5.rows) >= 2:
        if "待确认" in (tbl5.rows[1].cells[2].text if len(tbl5.rows[1].cells) > 2 else ""):
            replace_table_contents(tbl5, [
                ["序号", "事项", "状态", "回复"],
                ["1", "HTTP API Base URL", "待确认", ""],
                ["2", "MAC地址认证方式", "待确认", ""],
                ["3", "jiance检测项编码含义（1=?, 2=?）", "待确认", ""],
                ["4", "HTTP接口路径是否匹配", "待确认", ""],
                ["5", "生化分析仪API文档", "待确认", ""],
                ["6", "理化分析仪API文档", "待确认", ""],
                ["7", "RTSP视频流地址", "待确认", ""],
                ["8", "点云切面轮廓坐标格式", "待确认", ""],
                ["9", "交接任务超时时间", "待确认", ""],
            ], col_widths=[800, 2800, 1200, 2200])
            print("  [OK] Table 5: confirmation items -> HTTP context")
        else:
            warnings.append("Table with '序号' header found but doesn't look like confirmation table")
    else:
        warnings.append("Table 5 (confirmation) not found")

    # ================================================================
    # Save and report
    # ================================================================
    doc.save(DOC_PATH)
    print(f"\nDone. Warnings: {len(warnings)}")
    for w in warnings:
        print(f"  - {w}")


if __name__ == "__main__":
    main()
