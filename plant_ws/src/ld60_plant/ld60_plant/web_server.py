"""Static file server for the web flow editor (no caching, so edits show up on reload)."""
import functools
import http.server
import socketserver
import threading

import rclpy
from rclpy.node import Node


class _Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, fmt, *args):  # keep the console quiet
        pass


class _Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class WebServer(Node):
    def __init__(self):
        super().__init__('amr_web_server')
        port = int(self.declare_parameter('port', 8080).value)
        root = self.declare_parameter('root', '').value
        handler = functools.partial(_Handler, directory=root)
        self.httpd = _Server(('0.0.0.0', port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'AMR dashboard ready: open http://localhost:{port} in your Windows browser')

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
