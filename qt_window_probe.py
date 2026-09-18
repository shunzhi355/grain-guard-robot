from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication, QLabel

app = QApplication([])
app.setQuitOnLastWindowClosed(False)
window = QLabel("Qt display probe")
window.setWindowTitle("QT_DISPLAY_PROBE")
window.resize(400, 300)
window.show()
QTimer.singleShot(250, window.show)
QTimer.singleShot(8000, app.quit)
app.exec_()
