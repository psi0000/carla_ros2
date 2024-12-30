#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField,NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        self.K = None  # Camera intrinsic parameters
        self.R = np.eye(3)  # Vehicle rotation matrix
        self.T = np.zeros(3)  # Vehicle position
        self.depth_data = None  # Depth data

        # Point cloud publishers
        self.road_pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/carla/ego_vehicle/road_pointcloud',
            qos_profile=10
        )
        self.sidewalk_pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/carla/ego_vehicle/sidewalk_pointcloud',
            qos_profile=10
        )

        # Sensor subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odom_callback,
            qos_profile=10
        )
        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            '/carla/ego_vehicle/gnss',  # GNSS
            self.gnss_callback,
            qos_profile=10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/carla/ego_vehicle/imu',  # IMU
            self.imu_callback,
            qos_profile=10
        )
        self.depth_info_subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/depth_front/image',
            self.depth_info_callback,
            qos_profile=10
        )
        self.seg_subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/semantic_segmentation_front/image',
            self.seg_callback,
            qos_profile=10
        )
        self.cam_info_subscription = self.create_subscription(
            CameraInfo,
            '/carla/ego_vehicle/semantic_segmentation_front/camera_info',
            self.cam_info_callback,
            qos_profile=10
        )
    def gnss_callback(self, msg):
        # print(f"Received GNSS Data:\n"
        #       f"Latitude: {msg.latitude}\n"
        #       f"Longitude: {msg.longitude}\n"
        #       f"Altitude: {msg.altitude}")
        pass

    def imu_callback(self, msg):
        # print(f"Received IMU Data:\n"
        #       f"Orientation: [w: {msg.orientation.w}, x: {msg.orientation.x}, y: {msg.orientation.y}, z: {msg.orientation.z}]\n"
        #       f"Angular Velocity: [x: {msg.angular_velocity.x}, y: {msg.angular_velocity.y}, z: {msg.angular_velocity.z}]\n"
        #       f"Linear Acceleration: [x: {msg.linear_acceleration.x}, y: {msg.linear_acceleration.y}, z: {msg.linear_acceleration.z}]")
        pass
    def odom_callback(self, msg):
        # Update vehicle position and rotation matrix
        pos = msg.pose.pose.position
        self.T = np.array([pos.x, pos.y, pos.z])
        ori = msg.pose.pose.orientation
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        self.R = R.from_quat(quaternion).as_matrix()

    def depth_info_callback(self, msg):
        try:
            # Decode depth image
            depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            self.depth_data = depth_array
            print("Depth data updated.")
        except Exception as e:
            print(f"Failed to process depth data: {e}")

    def seg_callback(self, msg):
        try:
            # Process segmentation image
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
            segmentation_mask = array[:, :, 2]
            self.process_segmentation_image(segmentation_mask)
        except Exception as e:
            print(f"Failed to process segmentation image: {e}")

    def cam_info_callback(self, msg):
        self.K = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])
        print("Camera intrinsic parameters set.")

    def process_segmentation_image(self, segmentation_mask):
        try:
            ROAD_ID = 128
            SIDEWALK_ID = 244
            road_pixels = np.argwhere(segmentation_mask == ROAD_ID)
            sidewalk_pixels = np.argwhere(segmentation_mask == SIDEWALK_ID)

            if self.depth_data is None:
                print("Depth data not available yet.")
                return

            road_coordinates = self.pixel_to_3d(road_pixels, "road")
            sidewalk_coordinates = self.pixel_to_3d(sidewalk_pixels, "sidewalk")

            self.publish_pointcloud(road_coordinates, "road")
            self.publish_pointcloud(sidewalk_coordinates, "sidewalk")
        except Exception as e:
            print(f"Error processing segmentation image: {e}")

    def pixel_to_3d(self, pixel_coords, label):
        world_coords = []

        if self.K is None or self.depth_data is None:
            print("Camera parameters or depth data not available.")
            return []

        R_90 = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])
        corrected_R = R_90 @ self.R

        for v, u in pixel_coords:
            depth = self.depth_data[v, u]
            if depth <= 0.1 or depth > 100.0:  # Check valid depth range
                continue

            # Convert pixel coordinates to camera coordinates
            x = (u - self.K[0, 2]) * depth / self.K[0, 0]
            y = (v - self.K[1, 2]) * depth / self.K[1, 1]
            z = depth

            # Transform camera coordinates to world coordinates
            camera_coord = np.array([-x, -z, y])
            world_coord = corrected_R @ camera_coord + self.T
            world_coords.append([world_coord[0], world_coord[1], world_coord[2], label])

        return np.array(world_coords)

    def publish_pointcloud(self, coordinates, label):
        if len(coordinates) == 0:
            print(f"No valid points for {label}.")
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1)
        ]

        points = []
        for point in coordinates:
            x, y, z, _ = point
            if label == "road":
                r, g, b = 255, 0, 0  # Red for road
            elif label == "sidewalk":
                r, g, b = 0, 255, 0  # Green for sidewalk
            a = 255  # Alpha (transparency)
            rgba = (r << 24) | (g << 16) | (b << 8) | a
            points.append([x, y, 0.0, rgba])

        pointcloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_dense=True,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(points),
            data=np.array(points, dtype=np.float32).tobytes()
        )

        if label == "road":
            self.road_pointcloud_publisher.publish(pointcloud)
        elif label == "sidewalk":
            self.sidewalk_pointcloud_publisher.publish(pointcloud)


def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass

    sensor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
