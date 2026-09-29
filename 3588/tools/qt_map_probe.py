from PySide2.QtWidgets import QApplication, QLabel
import os, sys
app = QApplication(sys.argv)
w = QLabel("Qt window map probe")
w.setWindowTitle(f"QtMapProbe-{os.getpid()}")
w.resize(420, 220)
w.show()
print(f"probe_pid={os.getpid()} winId={int(w.winId())}", flush=True)
sys.exit(app.exec_())
