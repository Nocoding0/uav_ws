#!/usr/bin/env python3
"""Read-only color observation. Never publishes flight setpoints or calls FCU services."""
import json
import math
import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from simple_target_follow.color_detection import detect_color_target


class ColorTargetNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.processed = None
        self.previous_center = None
        self.confirmed = 0
        self.lower = tuple(rospy.get_param('~hsv_lower', [95, 100, 60]))
        self.upper = tuple(rospy.get_param('~hsv_upper', [130, 255, 255]))
        self.options = {name: float(rospy.get_param('~' + name, default)) for name, default in (
            ('minimum_area', 300), ('maximum_area_fraction', 0.5),
            ('minimum_fill_ratio', 0.5), ('ambiguity_ratio', 0.65))}
        self.max_age = float(rospy.get_param('~maximum_image_age', 0.4))
        self.required = int(rospy.get_param('~confirm_frames', 3))
        self.max_jump = float(rospy.get_param('~maximum_center_jump', 0.15))
        hz = float(rospy.get_param('~processing_rate', 10))
        if self.max_age <= 0 or self.required < 1 or self.max_jump <= 0 or hz <= 0:
            raise ValueError('invalid timing/confirmation parameters')
        self.status = rospy.Publisher('/target_follow/target', String, queue_size=1)
        self.annotated = rospy.Publisher('/target_follow/image', Image, queue_size=1)
        self.mask = rospy.Publisher('/target_follow/mask', Image, queue_size=1)
        self.subscriber = rospy.Subscriber(rospy.get_param('~image_topic', '/camera/color/image_raw'),
                                          Image, self.receive, queue_size=1, buff_size=2**24)
        self.timer = rospy.Timer(rospy.Duration(1.0 / hz), self.process)

    def receive(self, msg):
        with self.lock:
            self.latest = msg  # replace old input; do not queue inference work

    def publish_invalid(self, reason, stamp=0.0, frame=''):
        self.confirmed = 0
        self.previous_center = None
        self.status.publish(String(data=json.dumps(dict(
            valid=False, reason=reason, stamp=stamp, frame_id=frame))))

    def process(self, _event):
        with self.lock:
            msg = self.latest
        if msg is None:
            self.publish_invalid('no_image')
            return
        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or age < 0 or age > self.max_age:
            self.publish_invalid('stale_or_invalid_stamp', stamp, msg.header.frame_id)
            return
        if self.processed is not None and stamp <= self.processed:
            return  # repeated frames never count toward confirmation
        self.processed = stamp
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            target, mask, reason = detect_color_target(image, self.lower, self.upper, **self.options)
        except (CvBridgeError, cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(2, 'Color detection failed: %s', exc)
            self.publish_invalid('processing_error', stamp, msg.header.frame_id)
            return
        payload = dict(valid=False, reason=reason, stamp=stamp, frame_id=msg.header.frame_id)
        if target is None:
            self.confirmed = 0
            self.previous_center = None
        else:
            continuous = self.previous_center is not None and math.hypot(
                target.center[0] - self.previous_center[0],
                target.center[1] - self.previous_center[1]) <= self.max_jump
            self.confirmed = self.confirmed + 1 if continuous else 1
            self.previous_center = target.center
            payload.update(valid=self.confirmed >= self.required,
                           reason='confirmed' if self.confirmed >= self.required else 'confirming',
                           center_normalized=target.center, bbox=target.bbox,
                           area=target.area, fill_ratio=target.fill_ratio)
            x, y, w, h = target.bbox
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(image, payload['reason'], (x, max(20, y-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        self.status.publish(String(data=json.dumps(payload)))
        for publisher, data, encoding in ((self.annotated, image, 'bgr8'), (self.mask, mask, 'mono8')):
            if publisher.get_num_connections():
                output = self.bridge.cv2_to_imgmsg(data, encoding)
                output.header = msg.header
                publisher.publish(output)


if __name__ == '__main__':
    rospy.init_node('color_target')
    node = ColorTargetNode()
    rospy.spin()
