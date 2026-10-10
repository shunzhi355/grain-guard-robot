import cv2, http.server, threading, time

class MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/camera":
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.wfile.write(b"--frame\r\n\r\nCamera offline\r\n")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)
        try:
            while True:
                ok, frame = cap.read()
                if not ok: break
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(0.05)
        finally:
            cap.release()
    def log_message(self, *a): pass

server = http.server.HTTPServer(("0.0.0.0", 8554), MJPEGHandler)
print("MJPEG: http://192.168.1.111:8554/camera")
server.serve_forever()