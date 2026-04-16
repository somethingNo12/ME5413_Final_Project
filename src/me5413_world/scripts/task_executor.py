#!/usr/bin/env python3
"""
task_executor.py
ME5413 Final Project (AY25/26) - Phase-based Autonomous Navigation
"""

import math
import rospy
import actionlib

from std_msgs.msg import Bool, Int16, Int32
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import tf.transformations as tft


# Phase 1 waypoints: first-floor patrol path.
PHASE1_WPS = [
    (4.0, -0.6, 0.00),
    (3.62, 16.40, 1.57),
    (5.62, 16.40, 0.00),
    (5.62, -1.45, -1.57),
    (7.62, -1.45, 0.00),
    (7.62, 16.40, 1.57),
    (9.62, 16.40, 0.00),
    (9.62, -1.45, -1.57),
    (11.0, -1.45, 0.00),
    (12.60, 13.0, 1.57),
    (13.50, 16.40, -1.57),
    (13.50, -1.45, -1.57),
    (15.5, -1.45, 0.00),
    (15.50, 16.40, 1.57),
    (17.50, 16.40, 0.00),
    (17.50, -1.45, -1.57),
    (19.50, -1.45, 0.00),
    (19.50, 16.40, 1.57),
    (21.50, 16.40, 0.0),
    (21.50, -1.45, -1.57),
]

# Phase 2 waypoints: exit and ramp traversal.
PHASE2_WPS = [
    (8.0, -1.22, -0.785),
    (8.18, -3.5, 0.0),
    # (24.31, -3.29, 0.0),
    (28.7, -3.18, 0.0),
    (33.1, -4.79, 0.0),
    (36.3, -3.00, 0.0),
    (41.0, -3.88, 1.5536),
]

# Phase 3 waypoints: upper corridor traversal.
PHASE3_WPS = [
    # (41.72, 5.97, 1.5536),
    (41.84, 15.82, 1.5536),
    (38.665, 8.092, -2.3562),
]

# Phase 4 waypoints: cone/gap check branch.
PHASE4_LOWER = (37.487, 6.977, -2.1068)
PHASE4_MID_TRUE = (37.487, 8.0, 1.5776)
PHASE4_UPPER = (35.6, 12.3, 2.3562)
PHASE4_MID_FALSE = (33.906, 1.913, -2.6760)

# Phase 5 waypoints: per-room observation points.
PHASE5_WPS = [
    (30.7, 14.84, 3.14),
    (30.7, 10.30, 3.14),
    (30.7, 4.98,  3.14),
    (30.7, 0.06,  3.14),
]
# Final room target points once a match is found (x shifted by -5m).
PHASE5_MATCH_WPS = [(x - 4.0, y, yaw) for (x, y, yaw) in PHASE5_WPS]


def _make_goal(x, y, yaw=0.0, frame='map'):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal


def _make_pose_stamped(x, y, yaw=0.0, frame='map'):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    ps.pose.orientation = Quaternion(*q)
    return ps


class TaskExecutor:
    # Navigation and detection timing parameters.
    ARRIVAL_THRESHOLD = 0.8
    YAW_THRESHOLD     = 1.0   # 到达判定的最大朝向误差（rad，约23°）
    DEFAULT_TIMEOUT = 90.0
    EXIT_PASS_TIMEOUT = 10.0
    DETECT_WINDOW = 1.5
    ROOM_DETECT_TIMEOUT = 5.0

    def __init__(self):
        rospy.init_node('task_executor', anonymous=False)

        # Runtime state caches updated by topic callbacks.
        self.amcl_pose = None
        self.blockornot_votes = []
        self.leastcount_digit = None
        self.latest_detectnumber = None
        self.detectnumber_seq = 0

        legacy = rospy.get_param('~target_pose_topic', '')
        self.next_goal_topic = rospy.get_param('~next_goal_topic', legacy if legacy else '/waypoint/next_goal')

        # Core control topics.
        self.pub_phase = rospy.Publisher('/task_phase', Int32, queue_size=1, latch=True)
        self.pub_respawn = rospy.Publisher('/rviz_panel/respawn_objects', Int16, queue_size=1, latch=True)
        self.pub_unblock = rospy.Publisher('/cmd_unblock', Bool, queue_size=1)
        self.pub_next_goal = rospy.Publisher(self.next_goal_topic, PoseStamped, queue_size=10, latch=True)
        # Enables/disables the room-specific OCR node in Phase 5.
        self.pub_room_digit_enable = rospy.Publisher('/room_digit_detector_enable', Bool, queue_size=1, latch=True)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._amcl_pose_cb, queue_size=1)
        rospy.Subscriber('/blockornot', Bool, self._blockornot_cb, queue_size=10)
        rospy.Subscriber('/leastcount', Int32, self._leastcount_cb, queue_size=10)
        rospy.Subscriber('/detectnumber', Int32, self._detectnumber_cb, queue_size=10)

        rospy.loginfo('[TaskExecutor] Connecting to move_base...')
        self.mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mb.wait_for_server()
        rospy.loginfo('[TaskExecutor] move_base connected.')

        self._set_room_digit_detector(False)

    def _amcl_pose_cb(self, msg):
        self.amcl_pose = msg.pose.pose

    def _blockornot_cb(self, msg):
        self.blockornot_votes.append(msg.data)

    def _leastcount_cb(self, msg):
        self.leastcount_digit = int(msg.data)
        rospy.loginfo('[TaskExecutor] /leastcount updated: %d', self.leastcount_digit)

    def _detectnumber_cb(self, msg):
        self.latest_detectnumber = int(msg.data)
        self.detectnumber_seq += 1
        rospy.loginfo('[TaskExecutor] /detectnumber updated: %d', self.latest_detectnumber)

    def _publish_phase(self, phase):
        # /task_phase drives other nodes' behavior (detector activation, etc.).
        self.pub_phase.publish(Int32(data=int(phase)))
        rospy.loginfo('[TaskExecutor] PHASE %d', phase)

    def _navigate_to(self, wp, timeout=None, label='', publish_goal=True, check_yaw=False):
        # Send a move_base goal and treat arrival by AMCL distance (+ optional yaw) threshold.
        if timeout is None:
            timeout = self.DEFAULT_TIMEOUT
        x, y, yaw = wp
        tag = label or f'({x:.2f},{y:.2f})'
        rospy.loginfo('[TaskExecutor] -> %s', tag)

        if publish_goal:
            self.pub_next_goal.publish(_make_pose_stamped(x, y, yaw))
        self.mb.send_goal(_make_goal(x, y, yaw))

        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.amcl_pose is not None:
                dx = self.amcl_pose.position.x - x
                dy = self.amcl_pose.position.y - y
                xy_ok = math.sqrt(dx * dx + dy * dy) <= self.ARRIVAL_THRESHOLD

                yaw_ok = True
                if check_yaw and xy_ok:
                    q = self.amcl_pose.orientation
                    _, _, curr_yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                    yaw_err = abs(math.atan2(math.sin(curr_yaw - yaw),
                                             math.cos(curr_yaw - yaw)))
                    yaw_ok = yaw_err <= self.YAW_THRESHOLD
                    if not yaw_ok:
                        rospy.loginfo_throttle(1.0, '[TaskExecutor] waiting yaw %.2f→%.2f (err %.2f)',
                                               curr_yaw, yaw, yaw_err)

                if xy_ok and yaw_ok:
                    self.mb.cancel_goal()
                    rospy.loginfo('[TaskExecutor] reached %s', tag)
                    return True
            if rospy.Time.now() >= deadline:
                rospy.logwarn('[TaskExecutor] timeout %s', tag)
                self.mb.cancel_goal()
                return False
            rate.sleep()
        return False

    def _navigate_to_mb(self, wp, timeout=None, label=''):
        """
        Phase 5 专用：等待 move_base 返回 SUCCEEDED（自带 xy+yaw 容差判定）。
        不提前 cancel_goal，让 move_base 完整完成旋转对齐。
        """
        if timeout is None:
            timeout = self.DEFAULT_TIMEOUT
        x, y, yaw = wp
        tag = label or f'({x:.2f},{y:.2f})'
        rospy.loginfo('[TaskExecutor] -> %s (mb)', tag)

        self.pub_next_goal.publish(_make_pose_stamped(x, y, yaw))
        self.mb.send_goal(_make_goal(x, y, yaw))

        finished = self.mb.wait_for_result(rospy.Duration(timeout))
        if finished:
            state = self.mb.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('[TaskExecutor] reached %s (SUCCEEDED)', tag)
                return True
            else:
                rospy.logwarn('[TaskExecutor] %s ended state=%d', tag, state)
                return False
        else:
            rospy.logwarn('[TaskExecutor] timeout %s (mb)', tag)
            self.mb.cancel_goal()
            return False

    def _collect_blockornot(self):
        # Collect recent blockornot votes and return majority result.
        self.blockornot_votes = []
        rospy.sleep(self.DETECT_WINDOW)
        if not self.blockornot_votes:
            rospy.logwarn('[TaskExecutor] No /blockornot votes received, defaulting False')
            return False
        true_count = sum(1 for v in self.blockornot_votes if v)
        false_count = len(self.blockornot_votes) - true_count
        result = true_count > false_count
        rospy.loginfo('[TaskExecutor] blockornot: True=%d False=%d -> %s', true_count, false_count, result)
        return result

    def _set_room_digit_detector(self, enable):
        # Toggle the room detector externally via topic.
        self.pub_room_digit_enable.publish(Bool(data=bool(enable)))
        rospy.loginfo('[TaskExecutor] room digit detector %s', 'enabled' if enable else 'disabled')

    def _wait_for_detectnumber(self, timeout_sec):
        # Wait for a *new* detectnumber message based on sequence id.
        start_seq = self.detectnumber_seq
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.detectnumber_seq > start_seq:
                return self.latest_detectnumber
            if rospy.Time.now() >= deadline:
                return None
            rate.sleep()
        return None

    def run(self):
        # Initialization window for Gazebo, TF, AMCL and spawned objects.
        rospy.loginfo('[TaskExecutor] Waiting 3s for system to stabilise...')
        rospy.sleep(3.0)

        rospy.loginfo('[TaskExecutor] Spawning objects...')
        spawn_msg = Int16(data=1)
        self.pub_respawn.publish(spawn_msg)
        rospy.loginfo('[TaskExecutor] Waiting 8s for objects to appear...')
        rospy.sleep(8.0)

        # Phase 1: patrol and count digits for leastcount pipeline.
        self._publish_phase(1)
        for i, wp in enumerate(PHASE1_WPS):
            if rospy.is_shutdown():
                return
            self._navigate_to(wp, label=f'p1_{i+1}')

        # Phase 2: exit procedure with dedicated timeout at the pass-through point.
        self._publish_phase(2)
        for i, wp in enumerate(PHASE2_WPS):
            if rospy.is_shutdown():
                return
            t = self.EXIT_PASS_TIMEOUT if i == 1 else self.DEFAULT_TIMEOUT
            self._navigate_to(wp, timeout=t, label=f'p2_{i+1}')
            if i == 0:
                rospy.loginfo('[TaskExecutor] Publishing /cmd_unblock -> 10s window!')
                self.pub_unblock.publish(Bool(data=True))

        # Phase 3: corridor navigation.
        self._publish_phase(3)
        for i, wp in enumerate(PHASE3_WPS):
            if rospy.is_shutdown():
                return
            self._navigate_to(wp, label=f'p3_{i+1}')

        # Phase 4: branch path using blockornot majority vote.
        self._publish_phase(4)
        self._navigate_to(PHASE4_LOWER, label='p4_lower')
        cone_blocked = self._collect_blockornot()
        if cone_blocked:
            self._navigate_to(PHASE4_MID_TRUE, label='p4_mid_true')
            self._navigate_to(PHASE4_UPPER, label='p4_upper')
        else:
            self._navigate_to(PHASE4_MID_FALSE, label='p4_mid_false')

        # Phase 5 room loop:
        # 1) go to room observation point
        # 2) enable room detector and wait detectnumber
        # 3) compare with cached leastcount
        # 4) on match, go to corresponding x-5 target point and stop searching
        self._publish_phase(5)
        self._set_room_digit_detector(False)

        if self.leastcount_digit is None:
            rospy.logwarn('[TaskExecutor] /leastcount not received yet before Phase 5')
        else:
            rospy.loginfo('[TaskExecutor] Cached /leastcount=%d for Phase 5', self.leastcount_digit)

        matched_room = -1
        for i, wp in enumerate(PHASE5_WPS):
            if rospy.is_shutdown():
                return

            self._navigate_to_mb(wp, label=f'p5_room_{i+1}')
            self._set_room_digit_detector(True)
            detected_digit = self._wait_for_detectnumber(self.ROOM_DETECT_TIMEOUT)
            self._set_room_digit_detector(False)

            if detected_digit is None:
                rospy.logwarn('[TaskExecutor] No /detectnumber for room %d, trying next room', i + 1)
                continue
            if detected_digit < 0:
                rospy.logwarn('[TaskExecutor] Invalid detectnumber in room %d, trying next room', i + 1)
                continue
            if self.leastcount_digit is None:
                rospy.logwarn('[TaskExecutor] leastcount unknown, cannot match in room %d', i + 1)
                continue

            if detected_digit == self.leastcount_digit:
                matched_room = i
                rospy.loginfo('[TaskExecutor] Match in room %d: digit=%d', i + 1, detected_digit)
                self._navigate_to_mb(PHASE5_MATCH_WPS[i], label=f'p5_room_{i+1}_target')
                break

            rospy.loginfo('[TaskExecutor] Room %d mismatch: detected=%d leastcount=%d',
                          i + 1, detected_digit, self.leastcount_digit)

        if matched_room < 0:
            rospy.logwarn('[TaskExecutor] No matching room found in Phase 5')

        rospy.loginfo('[TaskExecutor] MISSION COMPLETE')


if __name__ == '__main__':
    try:
        TaskExecutor().run()
    except rospy.ROSInterruptException:
        pass
