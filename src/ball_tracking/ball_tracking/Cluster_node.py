#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2 as cv
import time
import math
import dv_processing as dv
from dbscan import DBSCAN
import matplotlib.pyplot as plt
import argparse
from datetime import timedelta


class EventCameraNode(Node):
    def __init__(self, capture):
        super().__init__('camera')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'observations', 1)
        self.subscriber = self.create_subscriber()

        self.publisher_.publish(msg)

        self.timer = self.create_timer(0.001, self.timer_callback)
        self.get_logger().info("Cluster node running.")




    def timer_callback(self):
        r,g,b = 0,0,0
        i=0
        events = self.capture.getNextEventBatch()
        if events is None:
            return

        resolution = (640, 480)

        filter = dv.noise.BackgroundActivityNoiseFilter(resolution, backgroundActivityDuration=timedelta(milliseconds=1))

        filter.accept(events)

        filtered = filter.generateEvents()

        frame = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
"""
        labels, _ = DBSCAN(filtered.coordinates(), eps=20, min_samples=15)
        for label in np.unique(labels):
            if label != -1:
                cluster_points = filtered.coordinates()[labels == label]
                for x, y in cluster_points:
                    if 0 <= x < self.resolution[0] and 0 <= y < self.resolution[1]:
                        frame[y, x] = (r%255, g%255, b%255)
                r+=50
                g+=20
                b+=10
                i+=1
                if i>4 :
                    for x, y in cluster_points:
                        if 0 <= x < self.resolution[0] and 0 <= y < self.resolution[1]:
                            frame[y, x] = (255, 255, 255)
"""

        for x, y in filtered.coordinates():
            if 0 <= x < self.resolution[0] and 0 <= y < self.resolution[1]:
                frame[y, x] = (255, 255, 255)

        cv.imshow("Preview", frame)
        cv.waitKey(1)

        ball_pose = None
        msg = Float64MultiArray()
        if ball_pose is not None:
            actual_time = time.time()
            local_pose = math.sqrt((ball_pose[0] - X_GUTTER_REF)**2 + (ball_pose[1] - Y_GUTTER_REF)**2) / GUTTER_LENGTH
            local_velocity = (local_pose - self.old_ball_position) / (actual_time - self.last_time)
            self.local_vel_act = self.alpha_vel * local_velocity + (1 - self.alpha_vel) * self.old_local_vel
            self.old_ball_position = local_pose
            self.old_local_vel = self.local_vel_act
            self.last_time = actual_time
            self.local_pose = local_pose
            self.x_init, self.y_init, self.r_init = ball_pose
            self.count_no_detection = 0
            msg.data = [local_pose, self.local_vel_act, self.position_des[0]]
        else:
            self.count_no_detection += 1
            if self.count_no_detection == 3:
                self.count_no_detection = 0
                msg.data = [self.local_pose, 0.0, self.position_des[0]]

        self.publisher_.publish(msg)

    def shutdown(self):
        self.get_logger().info("Shutting down, saving plots...")
        if self.time_values_plot:
            plt.figure(1)
            plt.subplot(211)
            plt.plot(self.time_values_plot, self.vel_glisse_plot)
            plt.title("ball velocity")
            plt.subplot(212)
            plt.plot(self.time_values_plot, self.pos_values_plot)
            plt.title("ball position")
            plt.savefig("Velocity.png")
            plt.show()
        cv.destroyAllWindows()
        rclpy.shutdown()


def main(args_ros=None):
    capture = dv.io.camera.open()
    if not capture.isEventStreamAvailable():
        raise RuntimeError("Camera does not provide an event stream.")

    rclpy.init(args=args_ros)
    node = EventCameraNode(capture)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
