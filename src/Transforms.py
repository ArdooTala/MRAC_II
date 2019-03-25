import numpy as np
import time


class Transformations:
    def __init__(self, marker_0, marker_1, marker_2):
        self.marker_0 = np.array(marker_0)
        self.marker_1 = np.array(marker_1)
        self.marker_2 = np.array(marker_2)

        # t0 = time.time()
        self.markers_in_robot = self.frame_from_points(self.marker_0, self.marker_1, self.marker_2)
        self.robot2marker_transform = np.linalg.inv(self.transform_from_plane(self.markers_in_robot))

    def frame_from_points(self, pt_0, pt_1, pt_2):
        u = pt_1 - pt_0
        x_vec = u / np.sqrt(np.sum(u ** 2))
        v = pt_2 - pt_0
        z_vec = np.cross(x_vec, v)
        z_vec = z_vec / np.sqrt(np.sum(z_vec ** 2))
        y_vec = np.cross(z_vec, x_vec)
        return x_vec, y_vec, z_vec, pt_0

    def transform_from_plane(self, plane):
        x_vec, y_vec, z_vec, origin = plane
        transformation = np.eye(4)
        transformation[:3, 0] = x_vec
        transformation[:3, 1] = y_vec
        transformation[:3, 2] = z_vec
        transformation[:3, 3] = origin
        return transformation

    def calculate_transformation(self, tvec_0, tvec_1, tvec_2):
        kinect2marker_transform = self.transform_from_plane(
            self.frame_from_points(tvec_0, tvec_1, tvec_2)
        )
        return kinect2marker_transform


# tr = Transformations([330, 330, 20], [730, -370, 20], [700, 300, 20])
