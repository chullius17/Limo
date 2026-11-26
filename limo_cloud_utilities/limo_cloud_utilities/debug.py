"""Optional legacy debug topics; drawing/encoding only runs for subscribers."""

import cv2
import numpy as np

from .processing.cloud_message import make_pointcloud2
from limo_cloud import Processor


class DebugOutputs(Processor):
    """Observe sampled images and the cloud before boardwalk classification.

    The optional rclpy node only publishes; it needs no executor or subscription
    callbacks. Its context is separate from the C++ host's rclcpp context.
    """

    def configure(self, parameters):
        self.enabled = bool(parameters.get('enable_debug_publications', False))
        self.quality = int(parameters.get('debug_jpeg_quality', 85))
        self.node = self.context = None
        if not 1 <= self.quality <= 100:
            raise ValueError('debug_jpeg_quality must be in [1, 100]')

    def _start(self, namespace):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage, PointCloud2
        from turbojpeg import TurboJPEG

        self.context = Context()
        rclpy.init(args=[], context=self.context)
        try:
            self.node = Node(
                'limo_cloud_utilities_debug', namespace=namespace,
                context=self.context, use_global_arguments=False)
            prefix = 'limo/cv_package/visual_ptcld/'
            self.debug_pub = self.node.create_publisher(
                CompressedImage, prefix + 'curb_points_debug/compressed', 1)
            self.lines_pub = self.node.create_publisher(
                CompressedImage, prefix + 'lines_and_curbs/compressed', 1)
            self.filter_pub = self.node.create_publisher(
                CompressedImage, prefix + 'blue_filter_debug/compressed', 1)
            self.before_pub = self.node.create_publisher(
                PointCloud2, prefix + 'points_before_kdtree', 1)
            self.jpeg = TurboJPEG()
        except Exception:
            self.close()
            raise

    def _header(self, frame):
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = frame.metadata['frame_id']
        header.stamp.sec, header.stamp.nanosec = divmod(frame.metadata['stamp_ns'], 10**9)
        return header

    def _jpeg(self, image, header):
        from sensor_msgs.msg import CompressedImage
        from turbojpeg import TJPF_BGR
        message = CompressedImage()
        message.header = header
        message.format = 'bgr8; jpeg compressed bgr8'
        message.data = self.jpeg.encode(image, quality=self.quality, pixel_format=TJPF_BGR)
        return message

    def process_cloud(self, frame):
        if not self.enabled:
            return
        if self.node is None:
            self._start(frame.metadata['node_namespace'])
        header = self._header(frame)
        if self.before_pub.get_subscription_count():
            points = np.column_stack((frame.cloud['x'], frame.cloud['y']))
            self.before_pub.publish(make_pointcloud2(
                points, frame.cloud['class_id'], header,
                frame.metadata['target_frame'], include_rgb=True))
        publish_debug = self.debug_pub.get_subscription_count() > 0
        publish_lines = self.lines_pub.get_subscription_count() > 0
        height, width = frame.labels.shape
        if publish_debug or publish_lines:
            image = np.zeros((height, width, 3), dtype=np.uint8)
            groups = dict((int(label), pixels)
                          for pixels, label in frame.scratch['visual/point_groups'])
            # Match the original draw order, including overlapping centroids.
            for label, color in ((2, (255, 255, 0)), (1, (255, 0, 0)),
                                 (5, (128, 0, 0)), (3, (255, 255, 255))):
                pixels = groups[label]
                if len(pixels):
                    image[tuple(pixels.T)] = color
            message = self._jpeg(image, header)
            if publish_debug:
                self.debug_pub.publish(message)
            if publish_lines:
                self.lines_pub.publish(message)
        if self.filter_pub.get_subscription_count():
            roi = np.zeros((height, width), dtype=bool)
            roi[int(height * frame.metadata['roi_y_min']):
                int(height * frame.metadata['roi_y_max']), :] = True
            original = (frame.labels == 1) & roi
            boundary = frame.images['visual/road'] != 0
            image = np.zeros((height + 24, width * 2, 3), dtype=np.uint8)
            before, after = image[24:, :width], image[24:, width:]
            before[frame.images['visual/background'] != 0] = (255, 255, 255)
            before[frame.images['visual/yellow'] != 0] = (255, 255, 0)
            after[:] = before
            before[original] = (255, 0, 0)
            after[boundary] = (255, 0, 0)
            for offset, title, mask in ((0, 'Before', original), (width, 'After', boundary)):
                cv2.putText(
                    image, '{}: {} blue'.format(title, np.count_nonzero(mask)),
                    (offset + 4, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
            self.filter_pub.publish(self._jpeg(image, header))

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context is not None:
            if self.context.ok():
                self.context.shutdown()
            self.context = None
