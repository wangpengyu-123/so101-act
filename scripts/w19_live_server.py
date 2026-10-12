#!/usr/bin/env python3
"""W19 相机实时画面浏览器查看（http://127.0.0.1:8899/）。

读 W19（by-id 稳定路径，可用环境变量 W19_DEV 覆盖），MJPG 1920x1080，MJPEG 流。
注意：不要 set(CAP_PROP_FPS)——实测设 30 会被固件砍到 15fps。
亮度增益：URL 加 ?bright=1.3（默认 1.0，范围 0.2~4.0，建议 1.2~1.5）。
"""
import os, cv2, http.server, threading, time, urllib.parse, json, socket

DEVICE = os.environ.get("W19_DEV", "/dev/v4l/by-id/<W19相机by-id路径>")
FALLBACK = '/dev/video2'
PORT = 8899

latest = [None]
stats = {'device': DEVICE, 'fps': 0.0, 'w': 0, 'h': 0, 'error': ''}


def capture():
    dev = DEVICE
    while True:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            dev = FALLBACK if dev == DEVICE else DEVICE
            stats['error'] = 'open failed: ' + dev
            time.sleep(2)
            continue
        stats['device'] = dev
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        # 不设 CAP_PROP_FPS（已知固件坑）
        t0 = time.time(); n = 0; t_last = t0; err_count = 0
        while True:
            ret, f = cap.read()
            if not ret or f is None:
                err_count += 1
                if err_count > 30:
                    break
                time.sleep(0.5)
                continue
            err_count = 0
            latest[0] = f
            n += 1
            now = time.time()
            if now - t_last >= 2.0:
                stats['fps'] = n / (now - t0)
                stats['w'], stats['h'] = f.shape[1], f.shape[0]
                stats['error'] = ''
                t0 = now; n = 0; t_last = now
        cap.release()
        stats['error'] = 'device lost, retrying'
        time.sleep(2)


def apply_gain(f, gain):
    if abs(gain - 1.0) < 1e-6:
        return f
    return cv2.convertScaleAbs(f, alpha=gain)


class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        q = dict(urllib.parse.parse_qsl(parsed.query))
        if parsed.path == '/':
            bright = q.get('bright', '1.0')
            html = f'''<html><head><title>W19 相机实时画面</title><meta charset="utf-8"></head>
<body style="margin:0;background:#111;color:#ddd;font-family:sans-serif">
<img id="v" src="/stream.mjpg?bright={bright}" style="width:100%;max-width:1200px">
<p style="padding:0 12px">W19 HD Webcam (0ac8:3460) · MJPG 1920x1080 · 当前亮度增益 bright={bright}（默认 1.0）</p>
<p style="padding:0 12px">亮度调节：在地址栏 URL 末尾加 <code>?bright=1.3</code> 后回车（建议 1.2~1.5，范围 0.2~4.0）。快捷：
<a href="/?bright=1.0" style="color:#6cf">1.0</a> ·
<a href="/?bright=1.3" style="color:#6cf">1.3</a> ·
<a href="/?bright=1.5" style="color:#6cf">1.5</a></p>
</body></html>'''
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode('utf-8'))
        elif parsed.path == '/stream.mjpg':
            try:
                gain = float(q.get('bright', '1.0'))
                gain = min(4.0, max(0.2, gain))
            except ValueError:
                gain = 1.0
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            last_id = None
            while True:
                f = latest[0]
                if f is not None and id(f) != last_id:
                    last_id = id(f)
                    g = apply_gain(f, gain)
                    ok, buf = cv2.imencode('.jpg', g, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ok:
                        try:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                            self.wfile.write(buf.tobytes())
                            self.wfile.write(b'\r\n')
                        except (BrokenPipeError, ConnectionResetError):
                            break
                time.sleep(0.01)
        elif parsed.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(stats).encode())
        else:
            self.send_response(404)
            self.end_headers()


if __name__ == '__main__':
    threading.Thread(target=capture, daemon=True).start()
    print(f'W19 MJPEG 服务已启动: http://127.0.0.1:{PORT}/  (bright 参数调节亮度)')
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        lan = s.getsockname()[0]
        s.close()
        print(f'局域网访问: http://{lan}:{PORT}/')
    except Exception:
        pass
    try:
        http.server.ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
    except OSError:
        print(f'端口 {PORT} 被占用，改用 {PORT - 1}')
        http.server.ThreadingHTTPServer(('0.0.0.0', PORT - 1), H).serve_forever()
