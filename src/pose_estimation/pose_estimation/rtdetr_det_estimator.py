import estimator
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv2
import numpy as np
import open3d as o3d
import itertools
import collections
from ultralytics import RTDETR
import copy
import sensor_msgs.msg as msgs


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


class RTDetrDetEstimator(estimator.Estimator):
    def __init__(self,logger):
        super().__init__(logger)

        self.det_model = RTDETR('rtdetr-x.pt')
        self.bottle_cls = 39
        self.down_sample_size = 8e-3
        self.model_path = '/storage/projects/AIMS5.0/stuff/uc6/bottle_large.pcd'

        logger.info(f'Load model {self.model_path}')
        ref_pcd = o3d.io.read_point_cloud(self.model_path)
        cnt = np.asarray(ref_pcd.points).shape[0]
        ref_pcd.colors = o3d.utility.Vector3dVector(np.repeat([[1,0,0]],cnt,axis = 0).astype(np.float32))
        self.ref_pcd = ref_pcd.voxel_down_sample(self.down_sample_size)

        logger.info('Estimator is ready')


    def estimate(self, color_msg, depth_msg, camera_msg):
        log = self.get_logger()
        log.info('Run estimation')

        color_img = np.frombuffer(color_msg.data, dtype = np.uint8).reshape(color_msg.height, color_msg.width,3)
        depth_img = np.frombuffer(depth_msg.data, dtype = np.uint16).reshape(depth_msg.height, depth_msg.width)
        Kdepth = camera_msg.k.reshape(3,3)

        results = self.det_model(color_img)
        det_result = results[0]
        boxes = det_result.boxes[det_result.boxes.cls == self.bottle_cls]

        if len(boxes) == 0:
            log.warn('No detection was found')
            return

        id = np.random.randint(0,len(boxes))

        log.info(f'Found {len(boxes)} bottles, selecting id = {id}')

        xyxy = boxes[id].xyxy.numpy().astype(np.uint32).flatten()
        hslice = slice(xyxy[1],xyxy[3])
        wslice = slice(xyxy[0],xyxy[2])

        diag_img_msg = copy.deepcopy(color_msg)
        diag_img_msg.data = det_result.plot().flatten().tolist()
 
        # create point cloud
        pts = cv2.rgbd.depthTo3d(depth_img, Kdepth)
        objpts = pts[hslice,wslice,:]

        verts = objpts.reshape((-1,3))
        idx = ~np.isnan(verts).any(axis=1)
        verts = verts[idx,:]
        color = color_img[hslice,wslice,:].reshape((-1,3))
        color = color[idx,:]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts)
        pcd.colors = o3d.utility.Vector3dVector(color/255)

        pcd_ds = pcd.voxel_down_sample(self.down_sample_size)

        # Rough estimation
        reg_p2p = o3d.pipelines.registration.registration_icp(
                self.ref_pcd, pcd_ds, 1, np.identity(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        print("")

        # Fine estimation
        reg_p2p = o3d.pipelines.registration.registration_icp(
                self.ref_pcd, pcd_ds, 0.01, reg_p2p.transformation,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        print("")

        tvec = reg_p2p.transformation[0:3,3]
        rvec,_ = cv2.Rodrigues(reg_p2p.transformation[0:3,0:3])
        rvec = rvec.T[0,:]

        ref_pcd_temp = copy.deepcopy(self.ref_pcd)
        ref_pcd_temp.transform(reg_p2p.transformation)
        #o3d.visualization.draw_geometries([pcd,ref_pcd_temp])

        pcd_points = np.concatenate([objpts.reshape((-1,3)),np.asarray(ref_pcd_temp.points,dtype=np.float32)])
        pcd_colors = np.concatenate([color_img[hslice,wslice,:].reshape((-1,3)),(np.asarray(ref_pcd_temp.colors) * 255).astype(np.uint8) ])

        pcd_msg = PCDToMessage(color_msg.header, pcd_points, pcd_colors)


        return diag_img_msg, pcd_msg


