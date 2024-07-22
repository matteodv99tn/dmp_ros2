#!/usr/bin/env python3

import os.path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from dmp_ros2_msgs.srv import RaycastFrameToMesh
import geometry_msgs

import numpy as np
import open3d as o3d
import math

from scipy.spatial.transform import Rotation as R



def extract_z_vector(pose) -> np.ndarray:
    return R.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]).as_matrix() @ np.array([0.0, 0.0, 1.0])



class ProjectorToMesh(Node):

    def __init__(self):
        super().__init__('frame_projector_to_mesh')
        def_mesh_file = os.path.join(
            get_package_share_directory("magician_material"), "mesh", "car_chassis.stl"
        )
        self.declare_parameter("mesh_file", def_mesh_file)
        self.declare_parameter("mesh_scaling", 0.001)

        mesh_file = self.get_parameter("mesh_file").get_parameter_value().string_value
        scaling = self.get_parameter("mesh_scaling").get_parameter_value().double_value
        self.get_logger().info("Loading mesh %s" % mesh_file)
        self.mesh = o3d.io.read_triangle_mesh(mesh_file).scale(scaling, np.zeros(3))
        self.get_logger().info("Mesh scaled by a factor %f" % scaling)

        t_mesh = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh)

        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(t_mesh)

        self.service = self.create_service(
            RaycastFrameToMesh, "intersection_on_mesh", self.mesh_intersection_callback
        )
        self.get_logger().info("Made available service '/intersection_on_mesh'")

    def mesh_intersection_callback(
        self, req: RaycastFrameToMesh.Request, resp: RaycastFrameToMesh.Response
    ):
        self.get_logger().info(
            "Processing intersections of %d frames" % len(req.target_frames)
        )
        if len(req.target_frames) == 0:
            return resp

        entries = list()
        directions = list()
        origins = list()
        frame: geometry_msgs.msg.Pose
        for frame in req.target_frames:
            vec = extract_z_vector(frame)
            orig = [frame.position.x, frame.position.y, frame.position.z]
            directions.append(vec)
            origins.append(orig)
            entries.append([*orig, *vec])

        rays = o3d.core.Tensor(entries, dtype=o3d.core.Dtype.Float32)
        cast_res = self.scene.cast_rays(rays)

        dir_x = np.array([1.0, 0.0, 0.0])
        t_hit = cast_res['t_hit'].numpy()

        for i in range(len(req.target_frames)):
            pose = geometry_msgs.msg.Pose()

            if (t_hit[i] == math.inf):
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
            else:
                translation = (origins[i] + directions[i] * t_hit[i])

                vers_k = directions[i]
                vers_i = dir_x - dir_x.dot(vers_k) * vers_k
                vers_i = vers_i / np.linalg.norm(vers_i)
                vers_j = np.cross(vers_k, vers_i)
                rot_mat = np.array([vers_i, vers_j, vers_k]).T
                q = R.from_matrix(rot_mat).as_quat()

                pose.position.x = translation[0]
                pose.position.y = translation[1]
                pose.position.z = translation[2]
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

            resp.projected_frames.append(pose)

        return resp



def main():
    rclpy.init()
    projector = ProjectorToMesh()
    rclpy.spin(projector)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
