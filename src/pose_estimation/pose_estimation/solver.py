import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sensor_msgs.msg as msgs
import collections
import rtdetr_det_estimator
import sam_seg_estimator
import yaml
import sys

class SolverSubscriber(Node):
    def __init__(self, config, estimator):
        super().__init__('solver')

        self.estimator = estimator
        self.config = config

        self.color_subscription = self.create_subscription(
            msgs.Image,
            '/camera/color/image_raw',
            self.color_callback,
            10)
        self.depth_subscription = self.create_subscription(
            msgs.Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            msgs.CameraInfo,
            '/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10)
        
        self.diag_image_publisher = self.create_publisher(msgs.Image,'/pose_estimation/diag/image', 10)
        self.diag_pcd_publisher = self.create_publisher(msgs.PointCloud2,'/pose_estimation/diag/points', 10)

        self.data = collections.OrderedDict()

    def put_data(self,ts,key,data):
        if(ts not in self.data):
            self.data[ts] = {}
        
        self.data[ts][key] = data

        for k,v in reversed(self.data.items()):
            if all(element in v for element in ['color','depth','ci']):
                self.data_ready_callback(k,v)
                del self.data[k]
                break

        # TODO: Remove all earlier entries


    def color_callback(self, msg):
        ts = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.get_logger().info(f'Got image info (ts: {msg.header.stamp.sec}.{msg.header.stamp.nanosec})')

        self.put_data(ts,'color',msg)

    def depth_callback(self, msg):
        ts = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.get_logger().info(f'Got depth info (ts: {msg.header.stamp.sec}.{msg.header.stamp.nanosec})')

        self.put_data(ts,'depth',msg)

    def camera_info_callback(self, msg: msgs.CameraInfo):
        ts = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.get_logger().info(f'Got camera info (ts: {msg.header.stamp.sec}.{msg.header.stamp.nanosec})')

        self.put_data(ts,'ci',msg)
        
    def data_ready_callback(self, ts, d):
        self.get_logger().info(f'Data is ready: {ts},{len(d)}')

        diag_image_msg, pcd_msg = self.estimator.estimate(d['color'], d['depth'], d['ci'])

        if diag_image_msg is not None:
            self.diag_image_publisher.publish(diag_image_msg)
            self.diag_pcd_publisher.publish(pcd_msg)

    

def main(args = None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Provide config.yaml path!')
        exit()
        
    config_path = sys.argv[1]

    print(f'Using {config_path}')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    #subscriber = SolverSubscriber(rtdetr_det_estimator.RTDetrDetEstimator(rclpy.logging.get_logger('estimator')))
    subscriber = SolverSubscriber(config,sam_seg_estimator.SAMSegmentEstimator(rclpy.logging.get_logger('estimator'),config))

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
