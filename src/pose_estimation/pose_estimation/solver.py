import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sensor_msgs.msg as msgs
import collections
import rtdetr_det_estimator
import sam_seg_estimator
import yaml
import copy
import sys
import pe_interface.msg as pe_msgs
import numpy as np

def PCDToMessage(header,pts,clr):
    print(f'pts: {pts.shape}, clr: {clr.shape}')

    shape = clr.shape
    clr = np.concatenate([clr,np.zeros((*shape[0:-1],1),dtype=np.uint8)],axis=-1)
    clr = np.frombuffer(clr.tobytes(),dtype=np.float32)
    clr = clr.reshape((*shape[0:-1],1))
    np.concatenate([pts,clr],axis=-1)

    pcd_msg = msgs.PointCloud2()
    pcd_msg.header = copy.deepcopy(header)
    if len(pts.shape) == 3:
        pcd_msg.height = pts.shape[0]
        pcd_msg.width = pts.shape[1]
    else:
        pcd_msg.height = 1
        pcd_msg.width = pts.shape[0]
    
    pcd_msg.is_bigendian = False
    pcd_msg.point_step = 16
    pcd_msg.row_step = pcd_msg.point_step * pcd_msg.width
    pcd_msg.is_dense = False
    pcd_msg.fields = [
        msgs.PointField(name = 'x', offset = 0, datatype = 7, count = 1),
        msgs.PointField(name = 'y', offset = 4, datatype = 7, count = 1),
        msgs.PointField(name = 'z', offset = 8, datatype = 7, count = 1),
        msgs.PointField(name = 'rgb', offset = 12, datatype = 7, count = 1)
    ]
    pcd_msg.data = list(np.concatenate([pts,clr],axis=-1).flatten().tobytes())

    return pcd_msg


class SolverSubscriber(Node):
    def __init__(self, config, estimator):
        super().__init__('solver')

        self.estimator = estimator
        self.config = config

        self.color_subscription = self.create_subscription(
            msgs.Image,
            config["topics"]["raw_image"],
            self.color_callback,
            10)
        self.depth_subscription = self.create_subscription(
            msgs.Image,
            config["topics"]["depth_image"],
            self.depth_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            msgs.CameraInfo,
            config["topics"]["camera_info"],
            self.camera_info_callback,
            10)
        
        if config['diag'] is True:
            self.diag_image_publisher = self.create_publisher(msgs.Image,'/pose_estimation/diag/image', 10)
            self.diag_pcd_publisher = self.create_publisher(msgs.PointCloud2,'/pose_estimation/diag/points', 10)

        self.pose_result = self.create_publisher(pe_msgs.Pose,'/pose_estimation/pose',10)

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

        result = self.estimator.estimate(d['color'], d['depth'], d['ci'])

        if self.config['diag'] is True:
            if self.estimator.get_diag_img() is not None:
                msg = copy.deepcopy(d['color'])
                msg.data = self.estimator.get_diag_img().flatten().tolist()
                self.diag_image_publisher.publish(msg)
            
            if self.estimator.get_diag_pcd() is not None:
                pcd,pcd_col = self.estimator.get_diag_pcd()
                msg = self.diag_pcd = PCDToMessage(d['color'].header, pcd, pcd_col)
                self.diag_pcd_publisher.publish(msg)

        if result is not None:
            r,t,diag = result
            print(f'R: {r.flatten().tolist()}; t: {t.tolist()}')

            msg = pe_msgs.Pose()
            msg.header = copy.deepcopy(d['color'].header)
            msg.r = r.flatten().tolist()
            msg.t = t.tolist()
            msg.inlier_rmse = diag.inlier_rmse
            msg.fitness = diag.fitness
   
            self.pose_result.publish(msg)

    

def main(args = None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Provide config.yaml path!')
        exit()
        
    config_path = sys.argv[1]

    print(f'Using {config_path}')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    print(f'Config: {config}')

    #subscriber = SolverSubscriber(rtdetr_det_estimator.RTDetrDetEstimator(rclpy.logging.get_logger('estimator')))
    subscriber = SolverSubscriber(config,sam_seg_estimator.SAMSegmentEstimator(rclpy.logging.get_logger('estimator'),config))

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
