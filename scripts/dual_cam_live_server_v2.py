#!/usr/bin/env python3
"""双相机预览 v2：global 订阅 ROS2 话题（不读 /dev/video10），wrist 读 W19 UVC。

    用途：录制时 lerobot 独占 /dev/video10，本服务 global 画面改从 ROS2 话题
    /ascamera/camera_publisher/rgb0/image 取帧，不与录制抢 /dev/video10。

    global 源: ROS2 话题 /ascamera/camera_publisher/rgb0/image (bgr8, 640x480, 实测)
    wrist  源: /dev/v4l/by-id/<W19相机by-id路径> (640x480 MJPG，不设 FPS)

    端口 8899，浏览器 http://127.0.0.1:8899/
    流: /stream_global.mjpg 、 /stream_wrist.mjpg ，均支持 ?bright=倍数
"""
import os
import http.server
import threading
import time

import cv2
import numpy as np

WRIST_DEV = os.environ.get("W19_DEV", "/dev/v4l/by-id/<W19相机by-id路径>")
ROS_TOPIC = '/ascamera/camera_publisher/rgb0/image'
W, H = 640, 480

latest = {'global': None, 'wrist': None}
seq = {'global': 0, 'wrist': 0}
stats = {'global': {'n': 0, 't': time.time()}, 'wrist': {'n': 0, 't': time.time()}}


def ros_global_thread():
    """订阅 ROS2 图像话题，把最新帧存到 latest['global']（BGR numpy）。"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init(args=['dual_cam_live_v2'])
    node = Node('dual_cam_live_v2')
    seen_enc = {}

    def cb(msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding != 'bgr8':
                if msg.encoding not in seen_enc:
                    seen_enc[msg.encoding] = True
                    print('[global] 未知 encoding=%r，按 BGR 直接使用' % msg.encoding, flush=True)
            latest['global'] = arr
            seq['global'] += 1
            stats['global']['n'] += 1
        except Exception as e:
            print('[global] ROS 帧解析失败: %r' % e, flush=True)

    node.create_subscription(Image, ROS_TOPIC, cb, 5)
    print('[global] 已订阅 ROS2 话题 %s（不读 /dev/video10）' % ROS_TOPIC, flush=True)
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)


def capture_wrist():
    cap = cv2.VideoCapture(WRIST_DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    # 按 W19 经验：不要 set FPS
    while True:
        ok, f = cap.read()
        if ok and f is not None:
            latest['wrist'] = f
            seq['wrist'] += 1
            stats['wrist']['n'] += 1
        else:
            time.sleep(0.3)


def stats_printer():
    while True:
        time.sleep(5)
        for n in ('global', 'wrist'):
            dt = time.time() - stats[n]['t']
            fps = stats[n]['n'] / dt if dt > 0 else 0.0
            print('[%s] 累计 %d 帧, 平均 %.1f fps' % (n, stats[n]['n'], fps), flush=True)


def query_param(path, key, default=None):
    if '?' not in path:
        return default
    qs = path.split('?', 1)[1].split('#', 1)[0]
    for kv in qs.split('&'):
        if '=' in kv and kv.split('=', 1)[0] == key:
            return kv.split('=', 1)[1]
    return default


def parse_float(path, key, default=1.0):
    try:
        return float(query_param(path, key, str(default)))
    except (TypeError, ValueError):
        return default


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            bg = parse_float(self.path, 'gbright', 1.0)
            bw = parse_float(self.path, 'wbright', 1.0)

            def block(title, stream, key, val, other_key, other_val):
                links = ('<a href="/?%s=1.2&amp;%s=%s">1.2</a> '
                         '<a href="/?%s=1.5&amp;%s=%s">1.5</a> '
                         '<a href="/?%s=2.0&amp;%s=%s">2.0</a> '
                         '<a href="/?%s=1.0&amp;%s=%s">正常</a>') % (
                    key, other_key, other_val,
                    key, other_key, other_val,
                    key, other_key, other_val,
                    key, other_key, other_val)
                return (
                    '<div style="flex:1;min-width:320px;background:#000">'
                    '<div style="padding:6px 12px;color:#ddd">%s &nbsp; 亮度: %s</div>'
                    '<img src="%s?bright=%s" style="width:100%%;display:block">'
                    '</div>' % (title, links, stream, val)
                )

            html = (
                '<html><head><title>dual cam live v2</title></head>'
                '<body style="margin:0;background:#111;color:#ccc;font-family:sans-serif">'
                '<div style="display:flex;flex-wrap:wrap;gap:10px">'
                + block('GLOBAL (ROS2 topic)', '/stream_global.mjpg', 'gbright', bg, 'wbright', bw)
                + block('WRIST (W19)', '/stream_wrist.mjpg', 'wbright', bw, 'gbright', bg)
                + '</div></body></html>'
            )
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode('utf-8'))
        elif path in ('/stream_global.mjpg', '/stream_wrist.mjpg'):
            name = 'global' if 'global' in path else 'wrist'
            bright = parse_float(self.path, 'bright', 1.0)
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            last_sent = -1
            while True:
                s = seq[name]
                if s != last_sent and latest[name] is not None:
                    f = latest[name]
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
                    last_sent = s
                time.sleep(0.02)
        else:
            self.send_response(404)
            self.end_headers()


threading.Thread(target=ros_global_thread, daemon=True).start()
threading.Thread(target=capture_wrist, daemon=True).start()
threading.Thread(target=stats_printer, daemon=True).start()
print('双相机预览 v2 已启动: http://127.0.0.1:8899/ (global=ROS2 话题, 不读 /dev/video10)', flush=True)
http.server.ThreadingHTTPServer(('127.0.0.1', 8899), Handler).serve_forever()
