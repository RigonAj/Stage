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

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("-lg", "--gutter_size", type=float, default=110)
parser.add_argument("-r", "--radius_ball", type=float, default=2.0)
args, _ = parser.parse_known_args()

Y_GUTTER_REF = 99
X_GUTTER_REF = 9
GUTTER_LENGTH = 110


class EventCameraNode(Node):
    def __init__(self, capture):

        super().__init__('camera')

        resolution = (640, 480)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'observations', 1)
        self.filter = dv.noise.BackgroundActivityNoiseFilter(resolution, backgroundActivityDuration=timedelta(milliseconds=1))
        self.visualizer = dv.visualization.EventVisualizer(resolution)
        self.capture = capture
        self.resolution = capture.getEventResolution()  # (width, height)

        # Camera intrinsics
        self.fx, self.fy = 301.536, 300.922
        self.cx, self.cy = 178.477, 122.871
        self.distortion = [-0.438, 0.946, 0.011, 0.009, -1.428]
        self.mtx = np.array([[self.fx, 0, self.cx],
                             [0, self.fy, self.cy],
                             [0, 0, 1]])
        self.geometry = dv.camera.CameraGeometry(self.fx, self.fy, self.cx, self.cy, self.resolution)

        # State
        self.no_detected = 0
        self.alpha_vel = 0.3
        self.local_velocity = 0.0
        self.local_pose = 0.0
        self.old_local_vel = 0.0
        self.local_vel_act = 0.0
        self.first_detection = True
        self.count_no_detection = 0
        self.position_des = [0.0, 0.8, 0.3, 0.1, 0.7, 0.3, 0.6]

        self.time_plot = 0.0
        self.pos_values_plot = []
        self.vel_values_plot = []
        self.vel_glisse_plot = []
        self.time_values_plot = []


        self.last_time = time.time()


        msg = Float64MultiArray()
        msg.data = [self.local_pose, self.local_velocity, 0.0 ]
        self.publisher_.publish(msg)


        cv.namedWindow("Preview", cv.WINDOW_NORMAL)


        self.timer = self.create_timer(0.001, self.timer_callback)
        self.get_logger().info("Event camera node running.")




    def timer_callback(self):
        r,g,b = 0,0,0
        i=0
        events = self.capture.getNextEventBatch()
        if events is None:
            return

        self.filter.accept(events)

        filtered = self.filter.generateEvents()

        frame = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)

        coords = filtered.coordinates().copy()

        xs = coords[:, 0]
        ys = coords[:, 1]
        print(len(coords))
        labels, _ = DBSCAN(coords, eps=30, min_samples=35)
        valid = labels != -1
        cluster_ids = labels[valid]

        if cluster_ids.size > 0:
            _, compact_ids = np.unique(cluster_ids, return_inverse=True)
            nb_clusters = cluster_ids.size

            r = np.arange(30, 30 + nb_clusters*100, 100) % 255
            g = np.arange(60, 60 + nb_clusters*(-150), -150) % 255
            b = np.arange(90, 90 + nb_clusters*200, 200) % 255


            colors = np.stack([r, g, b], axis=1).astype(np.uint8)

            frame[ys[valid], xs[valid]] = colors[compact_ids]

            unique_clusters = np.unique(compact_ids)

            circles = []

            for cluster_idx in unique_clusters:
                mask = compact_ids == cluster_idx
                xs_cluster = xs[valid][mask]
                ys_cluster = ys[valid][mask]

                x_mean, y_mean = np.mean(xs_cluster), np.mean(ys_cluster)
                scale = np.std(np.concatenate([xs_cluster - x_mean, ys_cluster - y_mean]))

                if scale < 1e-6:
                    continue
                xn = (xs_cluster - x_mean) / scale
                yn = (ys_cluster - y_mean) / scale

                A = np.column_stack([2*xn, 2*yn, np.ones_like(xn)])
                b_vec = xn**2 + yn**2

                sol, _, rank, sv = np.linalg.lstsq(A, b_vec, rcond=None)
                #print("sv normalisé:", sv)

                xc_n, yc_n, c_n = sol
                val = xc_n**2 + yc_n**2 + c_n

                if val <= 0:
                    continue

                radius_n = np.sqrt(val)

                xc = xc_n * scale + x_mean
                yc = yc_n * scale + y_mean
                radius = radius_n * scale

                distances = np.sqrt((xn - xc_n)**2 + (yn - yc_n)**2)

                # Ecart moyen relatif entre les distances et le rayon trouvé
                residual = np.mean(np.abs(distances - radius_n)) / radius_n

                #print(f"residual={residual:.3f}")


                if np.isfinite(xc) and np.isfinite(yc) and np.isfinite(radius) and 3 < radius < 60 and residual < 0.3:
                    cv.circle(frame, (int(xc), int(yc)), int(radius), (0, 255, 0), 2)



        input = self.visualizer.generateImage(events)

        output = self.visualizer.generateImage(filtered)


        # Concatenate the images into a single image for preview

        preview = cv.hconcat([input, output])
        cv.imshow("preview", frame)
        cv.waitKey(1)




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
