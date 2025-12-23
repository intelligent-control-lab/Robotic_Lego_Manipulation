import trimesh
from foundationpose.Utils import *
from foundationpose.estimater import *
from utils import *

class FoundationPoseEstimator:
    def __init__(self):
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        self.glctx = dr.RasterizeCudaContext()

    def locate(self, mesh, color_img, depth, mask, cam_K, track=False, est_refine_iter=5, track_refine_iter=2, vis_detection=False):
        if(not track):
            self.est = FoundationPose(model_pts=mesh.vertices, 
                                      model_normals=mesh.vertex_normals, 
                                      mesh=mesh, scorer=self.scorer, refiner=self.refiner, glctx=self.glctx)
            pose = self.est.register(K=cam_K, 
                                     rgb=color_img, 
                                     depth=depth, 
                                     ob_mask=mask, 
                                     iteration=est_refine_iter)
        else:
            pose = self.est.track_one(rgb=color_img, depth=depth, K=cam_K, iteration=track_refine_iter)

        if(vis_detection):
            aabb = mesh.bounding_box
            extents = aabb.extents
            to_origin = trimesh.transformations.translation_matrix(-aabb.centroid)
            bbox = np.stack([-extents/2, extents/2], axis=0).reshape(2,3)
            center_pose = pose@np.linalg.inv(to_origin)
            vis = draw_posed_3d_box(cam_K, img=color_img, ob_in_cam=center_pose, bbox=bbox)
            vis = draw_xyz_axis(color_img, ob_in_cam=center_pose, scale=0.1, K=cam_K, thickness=3, transparency=0, is_input_rgb=True)
            cv2.imshow('1', vis[...,::-1])
            cv2.waitKey(0)
        return pose # center of the brick on the top surface (excluding the top knobs)