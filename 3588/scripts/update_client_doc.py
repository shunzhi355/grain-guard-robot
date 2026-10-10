import docx
from docx.oxml.ns import qn

doc_path = r'E:\青赋驭境\项目\粮食扦样\grain_sampling_robot_software\docs\甲方接口文档.docx'
doc = docx.Document(doc_path)

# Guard: skip if already updated
existing_headings = [p.text.strip() for p in doc.paragraphs if p.style.name == 'Heading 1']
if any("点云切面轮廓数据格式" in h for h in existing_headings):
    print("Already updated - skipping.")
    exit(0)

# 1. Rename "7. 待甲方确认事项" -> "8. 待甲方确认事项"
for p in doc.paragraphs:
    if p.text.strip() == "7. 待甲方确认事项":
        p.clear()
        run = p.add_run("8. 待甲方确认事项")
        run.bold = True
        p.style = doc.styles['Heading 1']
        break

# 2. Find the heading element for "8." to insert before it
target_el = None
for p in doc.paragraphs:
    if p.text.strip().startswith("8. 待甲方确认事项"):
        target_el = p._element
        break
assert target_el is not None, "Target paragraph not found"

body = doc.element.body

# 3. Helper: make a heading
def mkh(text):
    p = docx.oxml.OxmlElement('w:p')
    pPr = docx.oxml.OxmlElement('w:pPr')
    ps = docx.oxml.OxmlElement('w:pStyle')
    ps.set(qn('w:val'), 'Heading1')
    pPr.append(ps)
    p.append(pPr)
    r = docx.oxml.OxmlElement('w:r')
    t = docx.oxml.OxmlElement('w:t')
    t.text = text
    r.append(t)
    p.append(r)
    return p

# 4. Helper: make a normal paragraph
def mkp(text):
    p = docx.oxml.OxmlElement('w:p')
    r = docx.oxml.OxmlElement('w:r')
    t = docx.oxml.OxmlElement('w:t')
    t.set(qn('xml:space'), 'preserve')
    t.text = text
    r.append(t)
    p.append(r)
    return p

# 5. Helper: make a 3-column table
def mktbl(headers, rows):
    tbl = docx.oxml.OxmlElement('w:tbl')
    # tblPr with width and borders
    tblPr = docx.oxml.OxmlElement('w:tblPr')
    tw = docx.oxml.OxmlElement('w:tblW')
    tw.set(qn('w:w'), '7600')
    tw.set(qn('w:type'), 'dxa')
    tblPr.append(tw)
    tblBorders = docx.oxml.OxmlElement('w:tblBorders')
    for bn in ['top','left','bottom','right','insideH','insideV']:
        b = docx.oxml.OxmlElement(f'w:{bn}')
        b.set(qn('w:val'), 'single')
        b.set(qn('w:sz'), '4')
        b.set(qn('w:space'), '0')
        b.set(qn('w:color'), '000000')
        tblBorders.append(b)
    tblPr.append(tblBorders)
    tbl.append(tblPr)
    # grid
    tg = docx.oxml.OxmlElement('w:tblGrid')
    for w in [2400, 1400, 3800]:
        gc = docx.oxml.OxmlElement('w:gridCol')
        gc.set(qn('w:w'), str(w))
        tg.append(gc)
    tbl.append(tg)
    # rows
    for ri, row in enumerate([headers] + rows):
        tr = docx.oxml.OxmlElement('w:tr')
        for ci, cell in enumerate(row):
            tc = docx.oxml.OxmlElement('w:tc')
            tcW = docx.oxml.OxmlElement('w:tcW')
            tcW.set(qn('w:w'), str([2400,1400,3800][ci]))
            tcW.set(qn('w:type'), 'dxa')
            tc.append(tcW)
            p = docx.oxml.OxmlElement('w:p')
            r = docx.oxml.OxmlElement('w:r')
            rPr = docx.oxml.OxmlElement('w:rPr')
            sz = docx.oxml.OxmlElement('w:sz')
            sz.set(qn('w:val'), '20')
            rPr.append(sz)
            if ri == 0:
                b = docx.oxml.OxmlElement('w:b')
                rPr.append(b)
            r.append(rPr)
            t = docx.oxml.OxmlElement('w:t')
            t.set(qn('xml:space'), 'preserve')
            t.text = cell
            r.append(t)
            p.append(r)
            tc.append(p)
            tr.append(tc)
        tbl.append(tr)
    return tbl

# 6. Helper: make a JSON code block paragraph (monospace)
def mkjson(text):
    p = docx.oxml.OxmlElement('w:p')
    for line in text.split('\n'):
        r = docx.oxml.OxmlElement('w:r')
        rPr = docx.oxml.OxmlElement('w:rPr')
        rf = docx.oxml.OxmlElement('w:rFonts')
        rf.set(qn('w:ascii'), 'Consolas')
        rf.set(qn('w:hAnsi'), 'Consolas')
        rPr.append(rf)
        sz = docx.oxml.OxmlElement('w:sz')
        sz.set(qn('w:val'), '18')
        rPr.append(sz)
        r.append(rPr)
        t = docx.oxml.OxmlElement('w:t')
        t.set(qn('xml:space'), 'preserve')
        t.text = line
        r.append(t)
        p.append(r)
        br = docx.oxml.OxmlElement('w:br')
        r.append(br)
    return p

# 7. Build elements (in REVERSE order since we insert before target)
json_block = (
    '{\n'
    '  "type": "contour",\n'
    '  "name": "1号廒间",\n'
    '  "height": 0.5,\n'
    '  "points": [\n'
    '    {"x": 12.3, "y": 5.1},\n'
    '    {"x": 12.5, "y": 5.3},\n'
    '    {"x": 12.8, "y": 5.6},\n'
    '    {"x": 13.1, "y": 5.9}\n'
    '  ],\n'
    '  "timestamp": "2026-07-15T10:35:00+00:00"\n'
    '}'
)

elements = [
    mkh("7. 点云切面轮廓数据格式"),
    mkp("粮堆点云经过切面提取后，上传到云端的是粮堆表面的外轮廓线坐标点，而非原始全量点云数据。原始点云数据量较大（每帧数万个点），轮廓线仅保留边界特征点（通常数十到数百个），大幅降低数据传输量和云端处理压力。"),
    mkp("上传数据字段说明："),
    mktbl(['字段', '类型', '说明'], [
        ['type', 'string', '固定值 "contour"，标识此数据为轮廓线'],
        ['name', 'string', '地图/廒间名称'],
        ['height', 'float', '切面高度（米）'],
        ['points', 'array', '轮廓点坐标数组，每个点为 {x, y}'],
        ['timestamp', 'string', 'ISO 8601 时间戳'],
    ]),
    mkp("JSON 示例："),
    mkjson(json_block),
    mkp("请甲方确认此格式是否满足云端接收要求。如有调整需求请反馈。"),
]

for el in elements:
    body.insert(list(body).index(target_el), el)

# 8. Add row to confirmations table
for t in doc.tables:
    first = t.rows[0].cells[0].text.strip()
    if first == '序号' and len(t.rows) >= 9:
        tr = docx.oxml.OxmlElement('w:tr')
        for ci, (text, w) in enumerate([('9','500'),('点云切面轮廓坐标格式','2800'),('待确认','1200'),('','2000')]):
            tc = docx.oxml.OxmlElement('w:tc')
            tcW = docx.oxml.OxmlElement('w:tcW')
            tcW.set(qn('w:w'), w)
            tcW.set(qn('w:type'), 'dxa')
            tc.append(tcW)
            p = docx.oxml.OxmlElement('w:p')
            r = docx.oxml.OxmlElement('w:r')
            t_elem = docx.oxml.OxmlElement('w:t')
            t_elem.set(qn('xml:space'), 'preserve')
            t_elem.text = text
            r.append(t_elem)
            p.append(r)
            tc.append(p)
            tr.append(tc)
        t._element.append(tr)
        break

doc.save(doc_path)
print("OK")
