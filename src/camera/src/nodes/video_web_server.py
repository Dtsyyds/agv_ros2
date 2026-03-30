import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response

class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')
        # 订阅 C++ 节点发布的彩色图像话题
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',  # 对应你 C++ 代码中的 topic 名称
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_frame = None

    def image_callback(self, msg):
        # 将 ROS Image 转换为 OpenCV Mat
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"转换图像失败: {e}")

    def generate_frames(self):
        """生成 MJPEG 流的生成器"""
        while True:
            if self.latest_frame is not None:
                # 将图像编码为 JPEG
                # 参数: 图像, 格式, 质量 (0-100, 越高越好但越大)
                ret, jpeg = cv2.imencode('.jpg', self.latest_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ret:
                    # 构造 HTTP 响应块
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            else:
                # 如果没有图像，稍微休眠一下避免 CPU 占用过高
                import time
                time.sleep(0.1)

def main():
    rclpy.init()
    node = VideoStreamNode()
    
    # 启动 Flask 服务器
    app = Flask(__name__)

    @app.route('/video_feed')
    def video_feed():
        return Response(node.generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/')
    def index():
        return "<h1>相机服务运行中</h1><p>请访问 <a href='/video_feed'>/video_feed</a> 查看视频流</p>"

    # 在单独线程运行 Flask，以免阻塞 ROS2 回调
    import threading
    threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000}).start()

    # 保持 ROS2 节点运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()