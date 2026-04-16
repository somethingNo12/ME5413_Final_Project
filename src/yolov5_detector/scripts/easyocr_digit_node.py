#!/usr/bin/env python3
"""
easyocr_digit_node.py
仿照 visual.py，用 EasyOCR 识别数字 + 激光测距定位 + EMA 去重。
激活控制：订阅 /task_phase，phase=1 激活，离开时发布 /leastcount。
"""

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Int32, String


class EasyOCRDigitNode:
    # ── 参数 ─────────────────────────────────────────────────────────────
    CONF_THRESH        = 0.99   # EasyOCR 置信度阈值
    DIAG_SHOW_PX       = 30     # 画框最小对角线像素（远处也显示）
    DIAG_COUNT_PX      = 60     # 定位/计数最小对角线像素（只有足够近才计入）
    MIN_ASPECT_RATIO   = 0.8    # 检测框最小宽高比（过窄说明数字被遮挡，如8→3、4→1）
    MIN_WIDTH_PX       = 25     # 检测框最小宽度像素（绝对宽度过小直接丢弃）
    EMA_ALPHA          = 0.15   # EMA 新观测权重（0.85×old + 0.15×new）
    DEDUP_DIST_M       = 0.9    # 同一数字两个检测位置超过此距离才视为不同箱子
    TURN_YAW_RATE_MAX  = 0.65   # 转弯角速度阈值（rad/s），超过此值暂停计数
    LIDAR_OFFSET       = 0.0    # 激光测距修正偏移（米）
    IMG_FRAME          = "front_camera"
    LIDAR_FRAME        = "tim551"
    MAP_FRAME          = "map"

    def __init__(self):
        self.bridge  = CvBridge()
        self.active  = False
        self.leastcount_published = False

        # digit → 已知唯一箱子位置列表，每个元素是 np.array([x, y])（EMA 滤波）
        # digit_counts[d] = 唯一箱子数 = len(digit_boxes[d])
        self.digit_boxes:  dict = {}   # {int: List[np.array([x, y])]}
        self.digit_counts: dict = {}   # {int: int}  只读，由 digit_boxes 派生

        self._display_frame = None   # 主循环显示用

        # 传感器数据
        self.img_curr    = None
        self.scan_curr   = None
        self.scan_params = None    # [angle_min, angle_max, angle_increment]
        self.yaw_rate    = 0.0     # 当前角速度（rad/s），转弯时暂停计数

        # 相机内参（阻塞等待一次）
        rospy.loginfo("[EasyOCRDigit] Waiting for camera_info...")
        cam_info = rospy.wait_for_message("/front/camera_info", CameraInfo, timeout=30.0)
        self.intrinsic  = np.array(cam_info.K).reshape(3, 3)
        self.img_frame  = cam_info.header.frame_id or self.IMG_FRAME

        # TF
        self.tf_listener = tf.TransformListener()

        # 加载 EasyOCR
        rospy.loginfo("[EasyOCRDigit] Loading EasyOCR model...")
        import easyocr
        self.reader = easyocr.Reader(["en"], gpu=True)
        rospy.loginfo("[EasyOCRDigit] EasyOCR ready.")

        # 订阅
        rospy.Subscriber("/front/image_raw",      Image,    self._img_cb,   queue_size=1, buff_size=2**24)
        rospy.Subscriber("/front/scan",           LaserScan,self._scan_cb,  queue_size=1)
        rospy.Subscriber("/task_phase",           Int32,    self._phase_cb, queue_size=10)
        rospy.Subscriber("/odometry/filtered",    Odometry, self._odom_cb,  queue_size=1)

        # 发布
        self.pub_leastcount = rospy.Publisher("/leastcount",              Int32,  queue_size=10, latch=True)
        self.pub_detections = rospy.Publisher("/easyocr_digit/detections", String, queue_size=10)
        self.pub_vis_image  = rospy.Publisher("/easyocr_digit/image",      Image,  queue_size=1)

        rospy.loginfo("[EasyOCRDigit] Node started.")

    # ── 回调 ─────────────────────────────────────────────────────────────

    def _img_cb(self, msg: Image):
        self.img_curr = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _scan_cb(self, msg: LaserScan):
        self.scan_curr   = np.array(msg.ranges, dtype=np.float32)
        self.scan_params = [msg.angle_min, msg.angle_max, msg.angle_increment]

    def _odom_cb(self, msg: Odometry):
        self.yaw_rate = msg.twist.twist.angular.z

    def _phase_cb(self, msg: Int32):
        phase = msg.data
        if phase == 1 and not self.active:
            rospy.loginfo("[EasyOCRDigit] Activated (phase=1)")
            self.active = True
            self.leastcount_published = False
            self.digit_boxes  = {}
            self.digit_counts = {}
        elif phase != 1 and self.active:
            rospy.loginfo("[EasyOCRDigit] Deactivated (phase=%d)", phase)
            self.active = False
            if not self.leastcount_published:
                self._publish_leastcount()
                self.leastcount_published = True
            self.digit_boxes  = {}
            self.digit_counts = {}

    # ── 定位：像素 → map 坐标 ────────────────────────────────────────────

    def _pixel_to_map(self, u: float, v: float) -> np.ndarray:
        """把像素坐标 (u,v) 转换为 map 帧坐标 (x,y)，失败返回 None。"""
        # 1. 相机内参反投影
        direction = np.dot(np.linalg.inv(self.intrinsic),
                           np.array([[u], [v], [1.0]]))
        p_cam = PoseStamped()
        p_cam.header.frame_id  = self.img_frame
        p_cam.pose.position.x  = float(direction[0])
        p_cam.pose.position.y  = float(direction[1])
        p_cam.pose.position.z  = float(direction[2])
        p_cam.pose.orientation.w = 1.0

        # 2. 转换到激光雷达系，求 yaw
        try:
            self.tf_listener.waitForTransform(
                self.LIDAR_FRAME, self.img_frame,
                rospy.Time(0), rospy.Duration(0.3))
            p_lidar = self.tf_listener.transformPose(self.LIDAR_FRAME, p_cam)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[EasyOCRDigit] TF cam→lidar: %s", e)
            return None

        yaw = np.arctan2(p_lidar.pose.position.y, p_lidar.pose.position.x)

        # 3. 激光测距
        if self.scan_curr is None or self.scan_params is None:
            return None
        a_min, _, a_inc = self.scan_params
        idx = int(round((yaw - a_min) / a_inc))
        idx = max(0, min(idx, len(self.scan_curr) - 1))
        dist = self.scan_curr[idx]
        if not np.isfinite(dist) or dist <= 0.05 or dist > 20.0:
            return None
        dist -= self.LIDAR_OFFSET

        angle = a_min + idx * a_inc
        p_in_lidar = PoseStamped()
        p_in_lidar.header.frame_id = self.LIDAR_FRAME
        p_in_lidar.header.stamp    = rospy.Time(0)
        p_in_lidar.pose.position.x = dist * np.cos(angle)
        p_in_lidar.pose.position.y = dist * np.sin(angle)
        p_in_lidar.pose.position.z = 0.0
        p_in_lidar.pose.orientation.w = 1.0

        # 4. 转换到 map
        try:
            self.tf_listener.waitForTransform(
                self.MAP_FRAME, self.LIDAR_FRAME,
                rospy.Time(0), rospy.Duration(0.3))
            p_map = self.tf_listener.transformPose(self.MAP_FRAME, p_in_lidar)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[EasyOCRDigit] TF lidar→map: %s", e)
            return None

        x, y = p_map.pose.position.x, p_map.pose.position.y
        if not (np.isfinite(x) and np.isfinite(y)):
            return None
        return np.array([x, y])

    # ── 空间去重 + EMA 位置滤波 ───────────────────────────────────────────

    def _update_ema(self, digit: int, pos: np.ndarray):
        """
        对每个数字维护一个唯一箱子位置列表：
        - 若新检测点距已有某个位置 ≤ DEDUP_DIST_M，视为同一箱子，EMA 更新位置
        - 否则视为新箱子，追加到列表
        digit_counts[digit] = 唯一箱子数
        """
        if digit not in self.digit_boxes:
            self.digit_boxes[digit] = [pos.copy()]
        else:
            # 找最近的已知位置
            dists = [np.linalg.norm(p - pos) for p in self.digit_boxes[digit]]
            min_idx = int(np.argmin(dists))
            if dists[min_idx] <= self.DEDUP_DIST_M:
                # 同一箱子，EMA 更新
                self.digit_boxes[digit][min_idx] = (
                    (1.0 - self.EMA_ALPHA) * self.digit_boxes[digit][min_idx]
                    + self.EMA_ALPHA * pos
                )
            else:
                # 新箱子
                self.digit_boxes[digit].append(pos.copy())

        self.digit_counts[digit] = len(self.digit_boxes[digit])

    # ── 发布结果 ──────────────────────────────────────────────────────────

    def _build_detections_str(self) -> str:
        if not self.digit_counts:
            return "[easyocr] active={} | no detections yet".format(self.active)
        items = "  ".join(
            "digit{}:{}hits".format(d, c) for d, c in sorted(self.digit_counts.items())
        )
        least = min(self.digit_counts, key=lambda d: (self.digit_counts[d], d))
        return "[easyocr] active={}  {}  | leastcount={}".format(
            self.active, items, least
        )

    def _publish_leastcount(self):
        if not self.digit_counts:
            rospy.logwarn("[EasyOCRDigit] No detections; publishing -1")
            self.pub_leastcount.publish(Int32(data=-1))
            return
        least = min(self.digit_counts, key=lambda d: (self.digit_counts[d], d))
        rospy.loginfo("[EasyOCRDigit] leastcount=%d (counts=%s)", least, self.digit_counts)
        self.pub_leastcount.publish(Int32(data=int(least)))

    # ── 主循环 ────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.active or self.img_curr is None:
                rate.sleep()
                continue

            img = self.img_curr.copy()

            # 转弯时 TF/激光时间戳错位，跳过计数（仍显示检测框）
            is_turning = abs(self.yaw_rate) > self.TURN_YAW_RATE_MAX

            result = self.reader.readtext(img, batch_size=2, allowlist="0123456789")
            vis = img.copy()

            for det in result:
                pts, text, conf = det
                if len(text) != 1:
                    continue

                # 计算检测框尺寸
                pt0, pt2 = np.array(pts[0]), np.array(pts[2])
                diag   = np.linalg.norm(pt2 - pt0)
                width  = abs(pt2[0] - pt0[0])
                height = abs(pt2[1] - pt0[1])
                aspect = width / height if height > 1e-3 else 0.0

                # 框太小则跳过（连显示都不值得）
                if diag < self.DIAG_SHOW_PX:
                    continue

                # 遮挡检测：宽度过窄或宽高比过小 → 数字被遮挡（如8→3、4→1）
                occluded = (width < self.MIN_WIDTH_PX or aspect < self.MIN_ASPECT_RATIO)

                # 画框：绿色=将计数，黄色=转弯暂停，橙色=被遮挡，灰色=太小/置信度不足
                will_count = (diag >= self.DIAG_COUNT_PX and conf >= self.CONF_THRESH
                              and not occluded and not is_turning)
                if is_turning and diag >= self.DIAG_COUNT_PX and conf >= self.CONF_THRESH and not occluded:
                    box_color = (0, 255, 255)   # 黄色：转弯中，暂停计数
                elif occluded:
                    box_color = (0, 165, 255)   # 橙色：遮挡警告
                elif will_count:
                    box_color = (0, 255, 0)     # 绿色：计数
                else:
                    box_color = (160, 160, 160) # 灰色：太小/置信度不足
                cv2.rectangle(vis,
                              (int(pts[0][0]), int(pts[0][1])),
                              (int(pts[2][0]), int(pts[2][1])),
                              box_color, 2)
                tag = '✓' if will_count else ('occluded' if occluded else '')
                cv2.putText(vis,
                            f"{text} {conf:.2f} {tag}",
                            (int(pts[0][0]), max(int(pts[0][1]) - 6, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)

                # 只有框足够大且置信度足够高才定位计数
                if not will_count:
                    continue

                digit = int(text)
                cx = (pts[0][0] + pts[2][0]) / 2.0
                cy = (pts[0][1] + pts[2][1]) / 2.0
                pos = self._pixel_to_map(cx, cy)
                if pos is None:
                    continue

                self._update_ema(digit, pos)

            # 左上角状态
            turn_info = f" TURNING({self.yaw_rate:.2f}r/s)" if is_turning else ""
            status = f"ACTIVE{turn_info}  digits:{sorted(self.digit_counts.keys())}"
            color  = (0, 255, 255) if is_turning else (0, 255, 0)
            cv2.putText(vis, status, (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 发布标注图像（用 rqt_image_view 订阅 /easyocr_digit/image 查看）
            self.pub_vis_image.publish(self.bridge.cv2_to_imgmsg(vis, encoding="bgr8"))

            self.pub_detections.publish(String(data=self._build_detections_str()))

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("easyocr_digit_node")
    node = EasyOCRDigitNode()
    node.run()
