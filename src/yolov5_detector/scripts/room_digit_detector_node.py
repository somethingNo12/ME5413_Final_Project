#!/usr/bin/env python3
"""
room_digit_detector_node.py

Phase-5 room digit detector:
- Enabled/disabled by /room_digit_detector_enable (Bool)
- While enabled, runs EasyOCR on /front/image_raw
- Publishes one detection result to /detectnumber (Int32)
"""

from collections import Counter

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32


class RoomDigitDetectorNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/front/image_raw")
        self.enable_topic = rospy.get_param("~enable_topic", "/room_digit_detector_enable")
        self.output_topic = rospy.get_param("~output_topic", "/detectnumber")
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.7))
        self.min_votes = int(rospy.get_param("~min_votes", 3))
        self.detect_timeout_sec = float(rospy.get_param("~detect_timeout_sec", 4.0))
        self.gpu = bool(rospy.get_param("~gpu", True))

        self.bridge = CvBridge()
        self.active = False
        self.published_this_session = False
        self.vote_counter = Counter()
        self.session_deadline = rospy.Time(0)

        rospy.loginfo("[RoomDigitDetector] Loading EasyOCR model...")
        import easyocr  # lazy import for startup logs

        self.reader = easyocr.Reader(["en"], gpu=self.gpu)
        rospy.loginfo("[RoomDigitDetector] EasyOCR ready.")

        self.pub_detectnumber = rospy.Publisher(self.output_topic, Int32, queue_size=10)
        rospy.Subscriber(self.enable_topic, Bool, self.enable_cb, queue_size=10)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_cb)
        rospy.loginfo("[RoomDigitDetector] Node started.")

    def enable_cb(self, msg):
        enabled = bool(msg.data)
        if enabled and not self.active:
            self.active = True
            self.published_this_session = False
            self.vote_counter = Counter()
            self.session_deadline = rospy.Time.now() + rospy.Duration(self.detect_timeout_sec)
            rospy.loginfo("[RoomDigitDetector] Enabled.")
            return

        if (not enabled) and self.active:
            self.active = False
            self.published_this_session = False
            self.vote_counter = Counter()
            rospy.loginfo("[RoomDigitDetector] Disabled.")

    def image_cb(self, msg):
        if not self.active or self.published_this_session:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[RoomDigitDetector] cv_bridge failed: %s", e)
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        try:
            results = self.reader.readtext(rgb, batch_size=2, allowlist="0123456789")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[RoomDigitDetector] OCR failed: %s", e)
            return

        for det in results:
            _, text, conf = det
            if len(text) != 1 or (not text.isdigit()):
                continue
            if float(conf) < self.conf_threshold:
                continue
            self.vote_counter[int(text)] += 1

        if not self.vote_counter:
            return

        best_digit, best_votes = max(
            self.vote_counter.items(),
            key=lambda kv: (kv[1], -kv[0]),  # tie-break: smaller digit wins
        )
        if best_votes >= self.min_votes:
            self.pub_detectnumber.publish(Int32(data=int(best_digit)))
            self.published_this_session = True
            rospy.loginfo("[RoomDigitDetector] Published /detectnumber=%d (votes=%d)",
                          best_digit, best_votes)

    def timer_cb(self, _event):
        if not self.active or self.published_this_session:
            return
        if rospy.Time.now() < self.session_deadline:
            return

        if self.vote_counter:
            best_digit, best_votes = max(
                self.vote_counter.items(),
                key=lambda kv: (kv[1], -kv[0]),
            )
            self.pub_detectnumber.publish(Int32(data=int(best_digit)))
            rospy.loginfo("[RoomDigitDetector] Timeout publish /detectnumber=%d (votes=%d)",
                          best_digit, best_votes)
        else:
            self.pub_detectnumber.publish(Int32(data=-1))
            rospy.logwarn("[RoomDigitDetector] Timeout with no detection, publish -1")

        self.published_this_session = True


if __name__ == "__main__":
    rospy.init_node("room_digit_detector_node")
    RoomDigitDetectorNode()
    rospy.spin()
