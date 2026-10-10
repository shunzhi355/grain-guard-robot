"""Dark industrial theme for the grain sampling robot UI.

Matches the grain-sampling-console.html design prototype (Open Design).
CSS variables → THEME_COLORS, HTML class selectors → QSS #objectName / type selectors.
"""

# ── Colour tokens ──────────────────────────────────────────
# Preserved existing keys. Added: bg_hover, border_dim, *_hover, on_bright.
THEME_COLORS = {
    "bg_darkest": "#0D1117",
    "bg_dark": "#161B22",
    "bg_medium": "#21262D",
    "bg_light": "#30363D",       # kept for backward compat; same as bg_hover/border_dim
    "bg_hover": "#30363D",        # --bg-hover
    "border": "#484F58",
    "border_dim": "#30363D",      # --border-dim
    "accent": "#2B579A",
    "accent_hover": "#3668B0",
    "success": "#2EA043",
    "success_hover": "#3AB550",   # oklch(--success l+0.07 c h)
    "warning": "#D4A72C",
    "warning_hover": "#E5B838",   # oklch(--warning l+0.06 c h)
    "danger": "#DA3633",
    "danger_hover": "#F85149",    # oklch(--danger l+0.07 c h)
    "info": "#58A6FF",
    "info_hover": "#79B8FF",      # lightened info
    "text_primary": "#E6EDF3",
    "text_secondary": "#8B949E",
    "text_disabled": "#585C63",   # kept for backward compat
    "on_bright": "#0D1117",       # --on-bright: dark text on warning/info solid bg
}

# ── Full QSS stylesheet ────────────────────────────────────
THEME_QSS = r"""
/* ============================================================
   GLOBAL — grain sampling robot UI (dark industrial)
   Matches grain-sampling-console.html Open Design prototype.
   ============================================================ */
* {
    font-family: "Noto Sans CJK SC", "AR PL UKai CN", "Noto Sans CJK HK", sans-serif;
    font-size: 9pt;
    color: #E6EDF3;
    background-color: #0D1117;
    selection-background-color: #2B579A;
    selection-color: #E6EDF3;
}

QMainWindow {
    background-color: #0D1117;
}

/* ============================================================
   LABEL — base + semantic variants mapped from HTML classes
   ============================================================ */
QLabel {
    background: transparent;
    border: none;
    color: #E6EDF3;
    font-size: 9pt;
}

/* --- Guide page (.guide-title / .guide-kicker / .guide-detail) --- */
QLabel#guide_title {
    font-size: 24pt;
    font-weight: bold;
    color: #E6EDF3;
}

QLabel#guide_kicker {
    font-size: 8pt;
    color: #8B949E;
}

QLabel#guide_detail {
    font-size: 9pt;
    color: #8B949E;
}

QLabel#guide_status {
    font-size: 9pt;
    font-weight: 600;
    color: #D4A72C;
    background-color: rgba(212, 167, 44, 0.14);
    border: 1px solid rgba(212, 167, 44, 0.45);
    border-radius: 999px;
    padding: 0px 10px;
    min-height: 24px;
}

QLabel#guide_status_running {
    font-size: 9pt;
    font-weight: 600;
    color: #4AC26B;
    background-color: rgba(46, 160, 67, 0.14);
    border: 1px solid rgba(46, 160, 67, 0.45);
    border-radius: 999px;
    padding: 0px 10px;
    min-height: 24px;
}

/* --- Page headers (.page-title / .page-sub) --- */
QLabel#page_title {
    font-size: 14pt;
    font-weight: bold;
    color: #E6EDF3;
}

QLabel#page_sub {
    font-size: 8pt;
    color: #8B949E;
}

/* --- Form labels (.field-label / .field-hint / .field-error) --- */
QLabel#field_label {
    font-size: 8pt;
    color: #8B949E;
}

QLabel#field_hint {
    font-size: 7pt;
    color: #8B949E;
}

QLabel#field_error {
    font-size: 8pt;
    color: #DA3633;
}

/* --- Status bar (.brand / .stat / .screen-name / .clock) --- */
QLabel#brand {
    font-size: 10pt;
    font-weight: bold;
    color: #E6EDF3;
}

QLabel#crumb {
    font-size: 10pt;
    color: #8B949E;
    font-weight: normal;
}

QLabel#stat_label {
    font-size: 9pt;
    color: #8B949E;
}

QLabel#stat_value {
    font-size: 9pt;
    color: #E6EDF3;
    font-weight: 600;
}

QLabel#clock {
    font-family: "Consolas", "SF Mono", "Menlo", monospace;
    font-size: 9pt;
    color: #8B949E;
}

/* --- Chips (.chip .chip-bio / .chip-phy) --- */
QLabel#chip_bio {
    font-size: 8pt;
    font-weight: 600;
    color: #4AC26B;
    background-color: rgba(46, 160, 67, 0.16);
    border-radius: 5px;
    padding: 2px 8px;
}

QLabel#chip_phy {
    font-size: 8pt;
    font-weight: 600;
    color: #58A6FF;
    background-color: rgba(88, 166, 255, 0.16);
    border-radius: 5px;
    padding: 2px 8px;
}

/* --- Pose tile labels (.pose-tile .k / .pose-tile .v) --- */
QLabel#pose_key {
    font-size: 8pt;
    color: #8B949E;
}

QLabel#pose_value {
    font-family: "Consolas", "SF Mono", "Menlo", monospace;
    font-size: 12pt;
    font-weight: 600;
    color: #E6EDF3;
}

/* --- Map tag --- */
QLabel#map_tag {
    font-size: 8pt;
    color: #8B949E;
    background-color: rgba(22, 27, 34, 0.92);
    border: 1px solid #484F58;
    border-radius: 6px;
    padding: 4px 8px;
}

/* --- Task card labels --- */
QLabel#task_id {
    font-family: "Consolas", "SF Mono", "Menlo", monospace;
    font-size: 10pt;
    font-weight: bold;
    color: #E6EDF3;
}

QLabel#task_line2 {
    font-size: 8pt;
    color: #E6EDF3;
}

QLabel#task_line3 {
    font-size: 8pt;
    color: #8B949E;
}

QLabel#task_ao {
    font-size: 8pt;
    color: #8B949E;
    background: transparent;
    border: 1px solid #484F58;
    border-radius: 6px;
    padding: 2px 8px;
}

/* --- Alarm bar --- */
QLabel#alarm_label {
    font-size: 8pt;
    font-weight: bold;
    color: #D4A72C;
}

QLabel#alarm_text {
    font-size: 8pt;
    color: #8B949E;
}

QLabel#alarm_warn {
    font-size: 8pt;
    color: #D4A72C;
}

QLabel#alarm_err {
    font-size: 8pt;
    color: #F0756E;
}

/* --- Modal labels --- */
QLabel#modal_title {
    font-size: 13pt;
    font-weight: bold;
    color: #E6EDF3;
}

QLabel#modal_text {
    font-size: 9pt;
    color: #8B949E;
}

/* --- Save note --- */
QLabel#save_note {
    font-size: 8pt;
    color: #4AC26B;
}

/* ============================================================
   PUSHBUTTON — 6 variants matching HTML .btn classes
   ============================================================ */

/* ── Default (.btn) ─────────────────────────────────────── */
QPushButton {
    background-color: #21262D;
    color: #E6EDF3;
    border: 1px solid #484F58;
    border-radius: 8px;
    padding: 0px 12px;
    min-height: 30px;
    font-size: 10pt;
    font-weight: bold;
}

QPushButton:hover {
    background-color: #30363D;
    border-color: #58A6FF;
}

QPushButton:pressed {
    background-color: #2B579A;
    border-color: #2B579A;
}

QPushButton:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Primary (.btn-primary) ─────────────────────────────── */
QPushButton#btn_primary {
    background-color: #2B579A;
    border-color: #2B579A;
    color: #FFFFFF;
}

QPushButton#btn_primary:hover {
    background-color: #3668B0;
    border-color: #3668B0;
}

QPushButton#btn_primary:pressed {
    background-color: #1F4378;
    border-color: #1F4378;
}

QPushButton#btn_primary:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Success (.btn-success) ─────────────────────────────── */
QPushButton#btn_success {
    background-color: #2EA043;
    border-color: #2EA043;
    color: #FFFFFF;
}

QPushButton#btn_success:hover {
    background-color: #3AB550;
    border-color: #3AB550;
}

QPushButton#btn_success:pressed {
    background-color: #22863A;
    border-color: #22863A;
}

QPushButton#btn_success:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Warning (.btn-warning) ─────────────────────────────── */
QPushButton#btn_warning {
    background-color: #D4A72C;
    border-color: #D4A72C;
    color: #0D1117;
}

QPushButton#btn_warning:hover {
    background-color: #E5B838;
    border-color: #E5B838;
}

QPushButton#btn_warning:pressed {
    background-color: #B89622;
    border-color: #B89622;
}

QPushButton#btn_warning:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Danger (.btn-danger) ───────────────────────────────── */
QPushButton#btn_danger {
    background-color: #DA3633;
    border-color: #DA3633;
    color: #FFFFFF;
}

QPushButton#btn_danger:hover {
    background-color: #F85149;
    border-color: #F85149;
}

QPushButton#btn_danger:pressed {
    background-color: #B52020;
    border-color: #B52020;
}

QPushButton#btn_danger:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Info (.btn-info) ───────────────────────────────────── */
QPushButton#btn_info {
    background-color: #58A6FF;
    border-color: #58A6FF;
    color: #0D1117;
}

QPushButton#btn_info:hover {
    background-color: #79B8FF;
    border-color: #79B8FF;
}

QPushButton#btn_info:pressed {
    background-color: #3A8AD9;
    border-color: #3A8AD9;
}

QPushButton#btn_info:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Navigation panel buttons (.nav-btn) ────────────────── */
QPushButton#nav_btn {
    min-height: 44px;
    max-height: 44px;
    font-size: 10pt;
    font-weight: 600;
    background-color: #21262D;
    border: 1px solid #484F58;
    border-radius: 8px;
    color: #E6EDF3;
    padding: 2px 6px;
    text-align: center;
}

QPushButton#nav_btn:hover {
    background-color: #30363D;
    border-color: #58A6FF;
}

QPushButton#nav_btn[active="true"] {
    background-color: #2B579A;
    border-color: #2B579A;
    color: #FFFFFF;
}

QPushButton#nav_btn:pressed {
    background-color: #1F4378;
    border-color: #1F4378;
}

/* ── Emergency stop (.estop) ────────────────────────────── */
QPushButton#estop {
    min-height: 56px;
    max-height: 56px;
    background-color: #DA3633;
    color: #FFFFFF;
    border: none;
    border-radius: 8px;
    font-size: 12pt;
    font-weight: bold;
}

QPushButton#estop:hover {
    background-color: #F85149;
}

QPushButton#estop:pressed {
    background-color: #B52020;
}

QPushButton#estop:disabled {
    background-color: #161B22;
    color: #8B949E;
    border-color: #30363D;
}

/* ── Manual mode switch (.manual_btn) — mirrors nav_btn, amber when active ─ */
QPushButton#manual_btn {
    min-height: 44px;
    max-height: 44px;
    font-size: 10pt;
    font-weight: 600;
    background-color: #21262D;
    border: 1px solid #484F58;
    border-radius: 8px;
    color: #E6EDF3;
    padding: 2px 6px;
    text-align: center;
}

QPushButton#manual_btn:hover {
    background-color: #30363D;
    border-color: #D4A72C;
}

QPushButton#manual_btn[active="true"] {
    background-color: #D4A72C;
    border-color: #D4A72C;
    color: #161B22;
}

QPushButton#manual_btn:pressed {
    background-color: #B8941F;
    border-color: #B8941F;
}

/* ── Guide page buttons (oversized) ─────────────────────── */
QPushButton#guide_btn {
    min-height: 36px;
    font-size: 11pt;
}

QPushButton#guide_finish_btn {
    min-height: 36px;
    font-size: 11pt;
}

/* ── Map item button ────────────────────────────────────── */
QPushButton#map_item_btn {
    min-height: 28px;
    font-size: 8pt;
    padding: 0px 10px;
}

/* ============================================================
   FRAME — .card styles
   ============================================================ */
QFrame#card {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 8px;
    padding: 6px;
}

QFrame#navpanel {
    background-color: #161B22;
    border-right: 1px solid #484F58;
}

QFrame#statusbar {
    background-color: #161B22;
    border-bottom: 1px solid #484F58;
}

QFrame#alarmbar {
    background-color: #161B22;
    border-top: 1px solid #484F58;
}

/* Task card frame */
QFrame#task_card {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 8px;
    padding: 6px 8px;
}

QFrame#task_card:hover {
    border-color: #58A6FF;
}

QFrame#task_card_selected {
    background-color: #161B22;
    border: 1px solid #2B579A;
    border-radius: 8px;
    padding: 6px 8px;
}

/* Map / slice panels */
QFrame#map_preview {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 8px;
}

QFrame#pose_tile {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 8px;
    padding: 6px 8px;
}

/* Modal / dialog */
QFrame#modal {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 10px;
    padding: 12px;
}

QFrame#modal_mask {
    background-color: rgba(2, 6, 12, 0.72);
}

QDialog {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 10px;
}

/* Depth visualization */
QFrame#depth_card {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 8px;
    padding: 6px;
}

/* ============================================================
   INPUT — QLineEdit / QSpinBox / QDoubleSpinBox (.input)
   ============================================================ */
QLineEdit,
QSpinBox,
QDoubleSpinBox {
    background-color: #21262D;
    border: 1px solid #484F58;
    border-radius: 6px;
    padding: 0px 8px;
    min-height: 32px;
    color: #E6EDF3;
    font-size: 10pt;
    selection-background-color: #2B579A;
}

QLineEdit:focus,
QSpinBox:focus,
QDoubleSpinBox:focus {
    border-color: #58A6FF;
}

QLineEdit[readOnly="true"] {
    color: #8B949E;
}

/* ============================================================
   COMBOBOX
   ============================================================ */
QComboBox {
    background-color: #21262D;
    border: 1px solid #484F58;
    border-radius: 6px;
    padding: 0px 8px;
    min-height: 32px;
    color: #E6EDF3;
    font-size: 10pt;
}

QComboBox:hover {
    border-color: #58A6FF;
}

QComboBox:focus {
    border-color: #58A6FF;
}

QComboBox QAbstractItemView {
    background-color: #161B22;
    border: 1px solid #484F58;
    color: #E6EDF3;
    selection-background-color: #2B579A;
    selection-color: #E6EDF3;
    outline: none;
    font-size: 10pt;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 32px;
    border-left: 1px solid #484F58;
    border-top-right-radius: 6px;
    border-bottom-right-radius: 6px;
    background-color: #21262D;
}

QComboBox::down-arrow {
    width: 10px;
    height: 10px;
}

/* ============================================================
   TAB WIDGET / TAB BAR — matches .tabs / .tab
   ============================================================ */
QTabWidget::pane {
    background-color: #0D1117;
    border: none;
    top: -1px;
}

QTabBar {
    background-color: #21262D;
    border: 1px solid #484F58;
    border-radius: 8px;
    padding: 4px;
}

QTabBar::tab {
    min-height: 28px;
    padding: 0px 14px;
    border: none;
    border-radius: 6px;
    background: transparent;
    color: #8B949E;
    font-size: 10pt;
    font-weight: 600;
    margin-right: 4px;
}

QTabBar::tab:selected {
    background-color: #2B579A;
    color: #FFFFFF;
}

QTabBar::tab:hover:!selected {
    background-color: #30363D;
    color: #E6EDF3;
}

QTabBar::tab:last {
    margin-right: 0px;
}

/* ============================================================
   PROGRESS BAR — matches .progress / .progress-track / .progress-fill
   ============================================================ */
QProgressBar {
    background-color: #21262D;
    border: 1px solid #484F58;
    border-radius: 999px;
    text-align: center;
    color: #E6EDF3;
    font-size: 9pt;
    min-height: 14px;
    max-height: 14px;
}

QProgressBar::chunk {
    background-color: #2B579A;
    border-radius: 999px;
}

/* ============================================================
   GROUP BOX
   ============================================================ */
QGroupBox {
    background-color: #161B22;
    border: 1px solid #484F58;
    border-radius: 8px;
    margin-top: 10px;
    padding: 8px 6px 6px 6px;
    font-size: 10pt;
    font-weight: bold;
    color: #E6EDF3;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 8px;
    color: #58A6FF;
    font-size: 10pt;
}

/* ============================================================
   SCROLL BAR
   ============================================================ */
QScrollBar:vertical {
    background: #0D1117;
    width: 14px;
    margin: 0px;
    border-radius: 7px;
}

QScrollBar::handle:vertical {
    background: #30363D;
    min-height: 30px;
    border-radius: 7px;
}

QScrollBar::handle:vertical:hover {
    background: #484F58;
}

QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical {
    height: 0px;
    border: none;
}

QScrollBar::add-page:vertical,
QScrollBar::sub-page:vertical {
    background: none;
}

QScrollBar:horizontal {
    background: #0D1117;
    height: 14px;
    margin: 0px;
    border-radius: 7px;
}

QScrollBar::handle:horizontal {
    background: #30363D;
    min-width: 30px;
    border-radius: 7px;
}

QScrollBar::handle:horizontal:hover {
    background: #484F58;
}

QScrollBar::add-line:horizontal,
QScrollBar::sub-line:horizontal {
    width: 0px;
    border: none;
}

QScrollBar::add-page:horizontal,
QScrollBar::sub-page:horizontal {
    background: none;
}

/* ============================================================
   SLIDER
   ============================================================ */
QSlider::groove:horizontal {
    background: #21262D;
    height: 8px;
    border-radius: 4px;
}

QSlider::handle:horizontal {
    background: #58A6FF;
    width: 20px;
    height: 20px;
    margin: -6px 0;
    border-radius: 10px;
}

QSlider::sub-page:horizontal {
    background: #2B579A;
    border-radius: 4px;
}

/* ============================================================
   CHECKBOX / RADIO BUTTON
   ============================================================ */
QCheckBox,
QRadioButton {
    color: #E6EDF3;
    font-size: 10pt;
    spacing: 8px;
}

QCheckBox::indicator,
QRadioButton::indicator {
    width: 18px;
    height: 18px;
    border: 2px solid #484F58;
    background-color: #161B22;
    border-radius: 4px;
}

QRadioButton::indicator {
    border-radius: 11px;
}

QCheckBox::indicator:checked,
QRadioButton::indicator:checked {
    background-color: #2B579A;
    border-color: #58A6FF;
}

QCheckBox::indicator:hover,
QRadioButton::indicator:hover {
    border-color: #58A6FF;
}

/* ============================================================
   STATUS BAR
   ============================================================ */
QStatusBar {
    background-color: #161B22;
    color: #8B949E;
    border: none;
    font-size: 9pt;
    min-height: 32px;
    padding: 0px 12px;
}

QStatusBar::item {
    border: none;
}

/* ============================================================
   MENU BAR / MENU
   ============================================================ */
QMenuBar {
    background-color: #161B22;
    color: #E6EDF3;
    border-bottom: 1px solid #30363D;
    font-size: 10pt;
}

QMenuBar::item:selected {
    background-color: #21262D;
}

QMenu {
    background-color: #161B22;
    color: #E6EDF3;
    border: 1px solid #484F58;
    border-radius: 6px;
    padding: 6px;
}

QMenu::item {
    padding: 4px 16px;
    border-radius: 4px;
    font-size: 10pt;
}

QMenu::item:selected {
    background-color: #2B579A;
}

QMenu::separator {
    height: 1px;
    background: #30363D;
    margin: 4px 12px;
}

/* ============================================================
   TOOLTIP
   ============================================================ */
QToolTip {
    background-color: #21262D;
    color: #E6EDF3;
    border: 1px solid #484F58;
    border-radius: 6px;
    padding: 4px 8px;
    font-size: 9pt;
}

/* ============================================================
   HEADER VIEW (table / tree)
   ============================================================ */
QHeaderView::section {
    background-color: #161B22;
    color: #8B949E;
    border: 1px solid #30363D;
    padding: 4px 8px;
    font-size: 9pt;
    font-weight: bold;
}

/* ============================================================
   TABLE VIEW
   ============================================================ */
QTableView {
    background-color: #161B22;
    color: #E6EDF3;
    border: 1px solid #484F58;
    border-radius: 6px;
    gridline-color: #30363D;
    selection-background-color: #2B579A;
    selection-color: #E6EDF3;
    font-size: 10pt;
}

QTableView::item {
    padding: 4px 8px;
}

QTableView::item:hover {
    background-color: #21262D;
}

/* ============================================================
   TREE VIEW
   ============================================================ */
QTreeView {
    background-color: #161B22;
    color: #E6EDF3;
    border: 1px solid #484F58;
    border-radius: 6px;
    selection-background-color: #2B579A;
    selection-color: #E6EDF3;
    font-size: 10pt;
}

QTreeView::item {
    padding: 4px 0px;
}

QTreeView::item:hover {
    background-color: #21262D;
}

/* ============================================================
   LIST VIEW
   ============================================================ */
QListView {
    background-color: #161B22;
    color: #E6EDF3;
    border: 1px solid #484F58;
    border-radius: 6px;
    selection-background-color: #2B579A;
    selection-color: #E6EDF3;
    font-size: 10pt;
    outline: none;
}

QListView::item {
    padding: 4px 8px;
}

QListView::item:hover {
    background-color: #21262D;
}

/* ============================================================
   SPLITTER
   ============================================================ */
QSplitter::handle {
    background-color: #30363D;
}

QSplitter::handle:horizontal {
    width: 2px;
}

QSplitter::handle:vertical {
    height: 2px;
}
"""
