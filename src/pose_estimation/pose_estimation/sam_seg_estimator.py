import estimator
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv2
import numpy as np
import open3d as o3d
import itertools
import collections
from ultralytics import RTDETR
from ultralytics import SAM
import copy
import sensor_msgs.msg as msgs
from timeit import default_timer as timer



class SAMSegmentEstimator(estimator.Estimator):
    def __init__(self,logger,config):
        super().__init__(logger,config)

        self.det_model = RTDETR('rtdetr-x.pt')
        self.seg_model = SAM('mobile_sam.pt')
        self.bottle_cls = [id for id,name in self.det_model.names.items() if name in ['bottle','cup']]
        self.down_sample_size = 8e-3
        self.model_path = config['model_pcd'] #'/storage/projects/AIMS5.0/stuff/uc6/bottle_large.pcd'

        logger.info(f'Using class ids {self.bottle_cls}')

        logger.info(f'Load model {self.model_path}')
        ref_pcd = o3d.io.read_point_cloud(self.model_path)
        cnt = np.asarray(ref_pcd.points).shape[0]
        ref_pcd.colors = o3d.utility.Vector3dVector(np.repeat([[1,0,0]],cnt,axis = 0).astype(np.float64))
        self.ref_pcd = ref_pcd.voxel_down_sample(self.down_sample_size)

        logger.info('Estimator is ready')

    def get_diag_img(self):
        return self.diag_img

    def get_diag_pcd(self):
        if self.diag_pcd is None:
            return None
        
        return self.diag_pcd,self.diag_pcd_col

    def create_diag_image(self,color_img, sam_result, results):
        seg_debug_img = color_img.copy()

        for id,(mask,result) in enumerate(zip(sam_result.masks,results)):
            mask = mask.data.cpu().numpy()[-1,:,:]

            blend = result[2].fitness
            seg_debug_img[mask,:] = (1 - blend) * seg_debug_img[mask,:] + (blend) * np.array([255,0,0])


        for id,(mask,result) in enumerate(zip(sam_result.masks,results)):
            crd = np.mean(mask.xy,axis=1).astype(np.int32).flatten().tolist()
            text = f"{id} - {result[2].fitness:0.2} - {result[2].inlier_rmse:0.3}"
            cv2.putText(seg_debug_img,org=crd,text=text,fontFace = 0, fontScale = 0.4, color = (0,255,0), thickness = 1)

        return seg_debug_img


    def estimate_pose_for_mask(self,pts,color_img,mask):
        objpts = pts[mask,:]
        color = color_img[mask,:]

        #remove NaNs
        verts = objpts.reshape((-1,3))
        idx = ~np.isnan(verts).any(axis=1)
        verts = verts[idx,:]
        color = color[idx,:]

        # create PCD
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts)
        pcd.colors = o3d.utility.Vector3dVector(color/255)

        pcd_ds = pcd.voxel_down_sample(self.down_sample_size)

        pts_mean = np.mean(np.asarray(pcd_ds.points),axis=0)
        initial_transform = np.block([[np.identity(3), np.asmatrix(pts_mean).T],[0,0,0,1]])

        reg_p2p = o3d.pipelines.registration.registration_icp(
            self.ref_pcd, pcd_ds, 1, initial_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))


        reg_p2p = o3d.pipelines.registration.registration_icp(
            self.ref_pcd, pcd_ds, 0.01, reg_p2p.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        

        tvec = reg_p2p.transformation[0:3,3]
        rvec,_ = cv2.Rodrigues(reg_p2p.transformation[0:3,0:3])
        rvec = rvec.T[0,:]
        return tvec,rvec,reg_p2p


    def estimate(self, color_msg, depth_msg, camera_msg):
        measureit_estimate = self.measureit('SAMSegmentEstimator')

        self.diag_img = None
        self.diag_pcd = None
        self.diag_pcd_col = None

        log = self.get_logger()
        log.info('Run estimation')

        color_img = np.frombuffer(color_msg.data, dtype = np.uint8).reshape(color_msg.height, color_msg.width,3)
        depth_img = np.frombuffer(depth_msg.data, dtype = np.uint16).reshape(depth_msg.height, depth_msg.width)
        Kdepth = camera_msg.k.reshape(3,3)

        results = self.det_model(color_img, classes = self.bottle_cls)
        det_result = results[0]
        boxes = det_result.boxes

        if len(boxes) == 0:
            log.warn('No detection was found')
            return None

        log.info(f'Found {len(boxes)} bottles')
        sam_result = self.seg_model.predict(color_img, bboxes = det_result.boxes.xyxy)[0]

        # create point cloud
        measureit_pcd = self.measureit("pointcloud matching")
        pts = cv2.rgbd.depthTo3d(depth_img, Kdepth)
        results = [self.estimate_pose_for_mask(pts,color_img,mask.numpy()) for mask in sam_result.masks.data]
        measureit_pcd.end()

        self.diag_img = self.create_diag_image(color_img, sam_result, results)

        #def generate_ref_point_cloud(transform):
        #    ref_pcd_temp = copy.deepcopy(self.ref_pcd)
        #    return ref_pcd_temp.transform(transform)

        #pcd_all = [generate_ref_point_cloud(reg_res.transformation) for _,_,reg_res in results] # 
        # TODO: point clouds

        results.sort(reverse = True, key = lambda x: x[2].fitness)
        measureit_estimate.end()

        return results


        # create point cloud
        # pts = cv2.rgbd.depthTo3d(depth_img, Kdepth)
        # objpts = pts[mask[-1,:,:],:]
        # color = color_img[mask[-1,:,:],:]

        # verts = objpts.reshape((-1,3))
        # idx = ~np.isnan(verts).any(axis=1)
        # verts = verts[idx,:]
        # color = color[idx,:]

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(verts)
        # pcd.colors = o3d.utility.Vector3dVector(color/255)

        # pcd_ds = pcd.voxel_down_sample(self.down_sample_size)

        # # Rough estimation
        # reg_p2p = o3d.pipelines.registration.registration_icp(
        #         self.ref_pcd, pcd_ds, 1, np.identity(4),
        #         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        # print(reg_p2p)
        # print("Transformation is:")
        # print(reg_p2p.transformation)
        # print("")

        # # Fine estimation
        # reg_p2p = o3d.pipelines.registration.registration_icp(
        #         self.ref_pcd, pcd_ds, 0.01, reg_p2p.transformation,
        #         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        # print(reg_p2p)
        # print("Transformation is:")
        # print(reg_p2p.transformation)
        # print("")

        # tvec = reg_p2p.transformation[0:3,3]
        # rvec = reg_p2p.transformation[0:3,0:3]

        # ref_pcd_temp = copy.deepcopy(self.ref_pcd)
        # ref_pcd_temp.transform(reg_p2p.transformation)

        # self.diag_pcd = np.concatenate([verts,np.asarray(ref_pcd_temp.points,dtype=np.float32)])
        # self.diag_pcd_col = np.concatenate([color,(np.asarray(ref_pcd_temp.colors) * 255).astype(np.uint8) ])

        # measureit_pcd.end()
        # measureit_estimate.end()

        # return rvec,tvec,reg_p2p


