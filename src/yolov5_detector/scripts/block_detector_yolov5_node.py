#!/usr/bin/env python3
import sys
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", category=FutureWarning, module="torch")

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class BlockDetectorYolov5Node:
    def __init__(self):
        self.bridge = CvBridge()

        self.phase_topic = rospy.get_param("~phase_topic", "/waypoint/phase")
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/blockornot")
        self.active_phase = str(rospy.get_param("~active_phase", "4"))
        self.stop_phase = str(rospy.get_param("~stop_phase", "5"))
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.6))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 5.0))
        self.infer_every_n_frames = int(rospy.get_param("~infer_every_n_frames", 1))

        default_yolov5_root = str(Path(__file__).resolve().parents[1] / "yolov5")
        default_weights = str(Path(default_yolov5_root) / "best.pt")
        self.yolov5_root = rospy.get_param("~yolov5_root", default_yolov5_root)
        self.weights_path = rospy.get_param("~weights_path", default_weights)

        self.detector_active = False
        self.latest_has_block = False
        self.frame_count = 0
        self.model = None

        self.pub_block = rospy.Publisher(self.output_topic, Bool, queue_size=10)
        self.sub_phase = rospy.Subscriber(self.phase_topic, String, self.phase_cb, queue_size=10)
        # 直接订阅 /task_phase (Int32)：phase=4 激活，其他值停止
        from std_msgs.msg import Int32
        rospy.Subscriber('/task_phase', Int32, self.task_phase_cb, queue_size=10)
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(0.1, self.publish_rate_hz)), self.publish_cb)

        self._load_model()
        rospy.loginfo("block_detector_yolov5_node started.")

    def _load_model(self):
        yolov5_root = Path(self.yolov5_root)
        weights = Path(self.weights_path)
        if not yolov5_root.exists():
            rospy.logerr("yolov5_root not found: %s", yolov5_root)
            return
        if not weights.exists():
            rospy.logerr("weights_path not found: %s", weights)
            return

        if str(yolov5_root) not in sys.path:
            sys.path.insert(0, str(yolov5_root))

        try:
            import torch  # noqa: PLC0415
            import pathlib  # noqa: PLC0415

            # Linux compatibility for checkpoints/caches produced on Windows.
            if sys.platform != "win32":
                pathlib.WindowsPath = pathlib.PosixPath

            # YOLOv5 local repo inference
            self.model = torch.hub.load(
                str(yolov5_root),
                "custom",
                path=str(weights),
                source="local",
                force_reload=True,
            )
            self.model.conf = self.conf_threshold
            self.model.iou = 0.45
            self.model.max_det = 20
            rospy.loginfo("YOLOv5 model loaded from %s", weights)
        except Exception as e:
            rospy.logerr("Failed to load YOLOv5 model: %s", e)
            self.model = None

    def task_phase_cb(self, msg):
        """激活条件：/task_phase = 4；其他值停止。"""
        if msg.data == 4:
            if not self.detector_active:
                rospy.loginfo("Block detector activated (task_phase=4)")
            self.detector_active = True
            self.latest_has_block = False
        else:
            if self.detector_active:
                rospy.loginfo("Block detector deactivated (task_phase=%d)", msg.data)
            self.detector_active = False
            self.latest_has_block = False

    def phase_cb(self, msg: String):
        phase = msg.data.strip()
        if phase == self.active_phase:
            if not self.detector_active:
                rospy.loginfo("Block detector enabled at phase %s", phase)
            self.detector_active = True
            self.latest_has_block = False
            return
        if phase == self.stop_phase:
            if self.detector_active:
                rospy.loginfo("Block detector disabled at phase %s", phase)
            self.detector_active = False
            self.latest_has_block = False
            return

    def image_cb(self, msg: Image):
        if not self.detector_active or self.model is None:
            return

        self.frame_count += 1
        if self.infer_every_n_frames > 1 and (self.frame_count % self.infer_every_n_frames) != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", e)
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        try:
            results = self.model(rgb, size=640)
            # best.pt is trained for traffic pillar only; any detection above conf is treated as block present.
            det = results.xyxy[0]  # tensor Nx6
            self.latest_has_block = (det is not None) and (len(det) > 0)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "YOLOv5 inference failed: %s", e)
            self.latest_has_block = False

    def publish_cb(self, _event):
        if not self.detector_active:
            return
        self.pub_block.publish(Bool(data=bool(self.latest_has_block)))


if __name__ == "__main__":
    rospy.init_node("block_detector_yolov5_node")
    BlockDetectorYolov5Node()
    rospy.spin()
