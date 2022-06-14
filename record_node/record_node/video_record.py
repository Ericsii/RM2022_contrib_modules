import rclpy
import rclpy.qos
from rclpy.node import Node
import threading
import cv_bridge
import cv2
import os
from datetime import datetime

from sensor_msgs.msg import Image

class VideoRecorder(Node):

    def __init__(self):
        super().__init__("video_recorder")

        self.declare_parameter("camera_name", "mv_camera")
        self.declare_parameter("best_effort_qos", False)
        self.declare_parameter("fps", 30)
        self.declare_parameter("video_dir", "")
        self.declare_parameter("frame_per_file", 1000)

        use_best_effort_qos = self.get_parameter("best_effort_qos").get_parameter_value().bool_value
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value
        self.frame_per_file = self.get_parameter("frame_per_file").get_parameter_value().integer_value
        self.camera_name = self.get_parameter("camera_name").get_parameter_value().string_value
        self.video_path = self.get_parameter("video_dir").get_parameter_value().string_value

        self.get_logger().info("camera_name: {}".format(self.camera_name))
        self.get_logger().info("best_effort_qos: {}".format(use_best_effort_qos))
        self.get_logger().info("fps: {}".format(self.fps))
        self.get_logger().info("video_dir: {}".format(self.video_path))
        self.get_logger().info("frame_per_file: {}".format(self.frame_per_file))

        if use_best_effort_qos:
            qos_profile = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=5, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
            qos_profile = rclpy.qos.HistoryPolicy.KEEP_LAST
            qos_profile.depth = 5
            self.img_sub = self.create_subscription(Image, self.camera_name + '/image_raw', self.img_callback, qos_profile)
        else:
            self.img_sub = self.create_subscription(Image, self.camera_name + '/image_raw', self.img_callback, 5)

        self.video_writer = None
        self.frame_cnt = 0

        self.is_recieved = False
        self.recv_img_lock = threading.Lock()
        self.recv_img = None
        self.recv_img_size = None

        self.timer = self.create_timer(1. / self.fps, self.timer_callback)

    def timer_callback(self):
        if not self.is_recieved:
            return

        if self.video_writer is None:
            now = datetime.now()
            now_fmt = now.strftime("%Y-%m-%d_%H:%M:%S")
            file_name = now_fmt+".mp4"
            self.get_logger().info("save video to: {}".format(file_name))
            self.video_writer = cv2.VideoWriter(os.path.join(self.video_path, file_name),
                                                cv2.VideoWriter_fourcc(*"mp4v"), 
                                                self.fps, self.recv_img_size)

        self.recv_img_lock.acquire()
        img = self.recv_img.copy()
        self.recv_img_lock.release()
        self.video_writer.write(img)
        self.frame_cnt += 1
        if self.frame_cnt > self.frame_per_file:
            self.video_writer.release()
            self.video_writer = None
            self.frame_cnt = 0

    def img_callback(self, msg):
        self.recv_img_lock.acquire()
        self.recv_img = cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.recv_img_size = (self.recv_img.shape[1], self.recv_img.shape[0])
        self.recv_img_lock.release()
        if not self.is_recieved:
            self.is_recieved = True


def main(args=None):
    rclpy.init(args=args)

    video_recorder = VideoRecorder()

    rclpy.spin(video_recorder)

    video_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()