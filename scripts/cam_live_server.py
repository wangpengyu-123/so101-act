#!/usr/bin/env python3
"""ASC60C 实时画面浏览器查看：读取 /dev/video10 并做 MJPEG 流，浏览器打开 http://127.0.0.1:8899/"""
import cv2, http.server, threading, time

latest = [None]

def query_param(path, key, default=None):
    """从 GET 路径中解析查询参数（如 /stream.mjpg?bright=1.2）。"""
    if '?' not in path:
        return default
    qs = path.split('?', 1)[1].split('#', 1)[0]
    for kv in qs.split('&'):
        if '=' in kv and kv.split('=', 1)[0] == key:
            return kv.split('=', 1)[1]
    return default

def capture():
    cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        ret, f = cap.read()
        if ret and f is not None:
            latest[0] = f
        else:
            time.sleep(0.5)

class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass
    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            bright = query_param(self.path, 'bright', '1.0')
            try:
                bright_val = float(bright)
                note = '' if bright_val == 1.0 else f' (亮度 x{bright})'
            except ValueError:
                bright_val, note = 1.0, ''
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write((
                '<html><head><title>ASC60C live</title></head>'
                '<body style="margin:0;background:#111;color:#ccc;font-family:sans-serif">'
                f'<div style="padding:6px 12px">ASC60C 实时画面{note} — 加亮: <a href="/?bright=1.2">1.2</a> '
                '<a href="/?bright=1.5">1.5</a> <a href="/?bright=2.0">2.0</a> '
                '<a href="/">正常</a></div>'
                f'<img src="/stream.mjpg?bright={bright_val}" style="width:100%;max-width:960px">'
                '</body></html>'
            ).encode('utf-8'))
        elif path == '/stream.mjpg':
            bright = query_param(self.path, 'bright', '1.0')
            try:
                bright = float(bright)
            except ValueError:
                bright = 1.0
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                f = latest[0]
                if f is not None:
                    if bright != 1.0:
                        f = cv2.convertScaleAbs(f, alpha=bright, beta=0)
                    ok, buf = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ok:
                        try:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                            self.wfile.write(buf.tobytes())
                            self.wfile.write(b'\r\n')
                        except (BrokenPipeError, ConnectionResetError):
                            break
                time.sleep(0.08)
        else:
            self.send_response(404)
            self.end_headers()

threading.Thread(target=capture, daemon=True).start()
print('MJPEG 服务已启动: http://127.0.0.1:8899/')
http.server.ThreadingHTTPServer(('127.0.0.1', 8899), H).serve_forever()
