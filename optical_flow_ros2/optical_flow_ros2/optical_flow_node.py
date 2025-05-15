import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
import cv2 as cv
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OpticalFlowNode(Node):
    def __init__(self):
        super().__init__('optical_flow_node')
        self.declare_parameter('video_source', 0)
        self.declare_parameter('show_debug', False)

        source = self.get_parameter('video_source').get_parameter_value().string_value or 0
        self.show_debug = self.get_parameter('show_debug').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, 'optical_flow/image', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.cap = cv.VideoCapture(source)
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('No se pudo leer del video source.')
            rclpy.shutdown()
            return

        self.prev_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        self.position = np.array([0.0, 0.0])
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.033, self.process_frame)
        self.tf_broadcaster = TransformBroadcaster(self)


    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        flow = cv.calcOpticalFlowFarneback(self.prev_gray, gray, None,
                                           0.5, 3, 15, 3, 5, 1.2, 0)
        self.prev_gray = gray
        vis = self.draw_flow(gray, flow)
        self.publish_image(vis)
        self.publish_odometry(flow)

        if self.show_debug:
            cv.imshow('Optical Flow', vis)
            cv.waitKey(1)

    def draw_flow(self, img, flow, step=16):
        h, w = img.shape[:2]
        y, x = np.mgrid[step//2:h:step, step//2:w:step].reshape(2,-1).astype(int)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        cv.polylines(vis, lines, 0, (0, 255, 0))
        for (x1, y1), (_x2, _y2) in lines:
            cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
        return vis

    def publish_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.img_pub.publish(msg)

    def publish_odometry(self, flow):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Básico 
        #
        fx = flow[:,:,0]
        fy = flow[:,:,1]
        mean_dx = np.mean(fx)
        mean_dy = np.mean(fy)


        # Avanzado deberes
        # fx = flow[:, :, 0]
        # fy = flow[:, :, 1]
        # v = np.sqrt(fx**2 + fy**2)

        # # Filtramos puntos con muy poco movimiento (ruido)
        # mask = v > 0.2  # Puedes probar también con 0.3 o 0.5
        # if np.any(mask):
        #     mean_dx = np.mean(fx[mask])
        #     mean_dy = np.mean(fy[mask])
        # else:
        #     mean_dx = 0.0
        #     mean_dy = 0.0


        scale = 0.05
        dx = -mean_dx * scale
        dy = -mean_dy * scale

        self.position += np.array([dx, dy])

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        quat = tf_transformations.quaternion_from_euler(0, 0, 0)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        odom_msg.twist.twist.linear.x = dx / dt
        odom_msg.twist.twist.linear.y = dy / dt

        self.odom_pub.publish(odom_msg)

        # Publicar TF de odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.position[0]
        tf_msg.transform.translation.y = self.position[1]
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
