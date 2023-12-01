#!/usr/bin/env python3
import tf2_ros
import numpy as np
import ros_numpy
import rospy
from cv_bridge import CvBridge
from ifm3d_ros_msgs.msg import Intrinsics, RGBInfo
from mag_common_py_libs.transformation import Transformation
from scipy import ndimage
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField


def intrinsic_projection(intrinsicModelID, intrinsicModelParameters,
                         width, height):
    """
    Evaluate intrinsic calibration model parameter and return unit vectors \
        in the optical reference system.

    Args:
        intrinsicModelID (int): intrinsic model id
        intrinsicModelParameters (numpy array): intrinsics model parameter as \
            returned from the camera (as part of the image chunk)
        width (int): image width
        height (int): image height

    Raises
    ------
        RuntimeError: raises Runtime error if modelID doesn't exist

    Returns
    -------
        tuple: direction vector in the optical reference system

    """
    if intrinsicModelID == 0:  # Bouguet model
        fx, fy, mx, my, alpha, k1, k2, k3, k4, k5 = \
            intrinsicModelParameters[:10]
        iy, ix = np.indices((height, width))
        cx = (ix + 0.5 - mx) / fx
        cy = (iy + 0.5 - my) / fy
        cx -= alpha * cy
        r2 = cx ** 2 + cy ** 2
        fradial = 1 + r2 * (k1 + r2 * (k2 + r2 * k5))
        h = 2 * cx * cy
        tx = k3 * h + k4 * (r2 + 2 * cx ** 2)
        ty = k3 * (r2 + 2 * cy ** 2) + k4 * h
        dx = fradial * cx + tx
        dy = fradial * cy + ty
        fnorm = 1 / np.sqrt(dx ** 2 + dy ** 2 + 1)
        vx = fnorm * dx
        vy = fnorm * dy
        vz = fnorm
        return vx, vy, vz
    elif intrinsicModelID == 2:  # fish eye model
        fx, fy, mx, my, alpha, k1, k2, k3, k4, theta_max = \
            intrinsicModelParameters[:10]
        iy, ix = np.indices((height, width))
        cx = (ix + 0.5 - mx) / fx
        cy = (iy + 0.5 - my) / fy
        cx -= alpha * cy
        theta_s = np.sqrt(cx ** 2 + cy ** 2)
        phi_s = np.minimum(theta_s, theta_max)
        p_radial = 1 + phi_s ** 2 * (
            k1 + phi_s ** 2 * (k2 + phi_s ** 2 * (k3 + phi_s ** 2 * k4))
        )
        theta = theta_s * p_radial
        theta = np.clip(
            theta, 0, np.pi
        )  # -> avoid surprises at image corners of extreme fisheye effect
        vx = np.choose((theta_s > 0), (0, (cx / theta_s) * np.sin(theta)))
        vy = np.choose((theta_s > 0), (0, (cy / theta_s) * np.sin(theta)))
        vz = np.cos(theta)
        return vx, vy, vz
    else:
        raise RuntimeError('Unknown model %d' % intrinsicModelID)


def rot_mat(r, order=(0, 1, 2)):
    R = np.eye(3)
    for i in order:
        lr = np.eye(3)
        lr[(i + 1) % 3, (i + 1) % 3] = np.cos(r[i])
        lr[(i + 2) % 3, (i + 2) % 3] = np.cos(r[i])
        lr[(i + 1) % 3, (i + 2) % 3] = -np.sin(r[i])
        lr[(i + 2) % 3, (i + 1) % 3] = np.sin(r[i])
        R = R.dot(lr)
    return R


def translate(pcd, X, Y, Z) -> np.ndarray:
    return np.array(pcd) + np.array((X, Y, Z))[..., np.newaxis]


def rotate_xyz(pcd, rotX, rotY, rotZ) -> np.ndarray:
    return rot_mat(np.array((rotX, rotY, rotZ))).dot(pcd)


def rotate_zyx(pcd, rotX, rotY, rotZ) -> np.ndarray:
    return rot_mat(np.array((rotX, rotY, rotZ)), order=(2, 1, 0)).dot(pcd)


def inverse_intrinsic_projection(pcd, invIntrinsic2D, modelID2D):
    """
    Inverse intrinsic projection model evaluation.

    Args:
        pcd (numpy array): point cloud data
        invIntrinsic2D (numpy array): inverse intrinsics model parameters
        modelID2D (int): model ID
        extrinsic2D (numpy array): extrinsic model parameters: [translation, \
            rotation]

    Raises
    ------
        RuntimeError: raised if model ID is not valid

    Returns
    -------
        tuple: image coordinates in pixel coordinate system: upper left \
            corner is (0,0)

    """
    tz = np.maximum(0.001, pcd[2, :])
    ixn = pcd[0, :] / tz
    iyn = pcd[1, :] / tz

    # apply distortion
    if modelID2D in [0, 1]:  # Bouguet model
        fx, fy, mx, my, alpha, k1, k2, k3, k4, k5 = invIntrinsic2D[:10]

        rd2 = ixn ** 2 + iyn ** 2
        radial = rd2 * (k1 + rd2 * (k2 + rd2 * k5)) + 1
        ixd = ixn * radial
        iyd = iyn * radial
        if k3 != 0 or k4 != 0:
            h = 2 * ixn * iyn
            tangx = k3 * h + k4 * (rd2 + 2 * ixn ** 2)
            tangy = k3 * (rd2 + 2 * iyn ** 2) + k4 * h
            ixd += tangx
            iyd += tangy
        # transform to imager
        ix = ((fx * (ixd + alpha * iyd)) + mx) - 0.5
        iy = ((fy * (iyd)) + my) - 0.5

    elif modelID2D in [2, 3]:  # fish eye model
        fx, fy, mx, my, alpha, k1, k2, k3, k4, theta_max = invIntrinsic2D[:10]

        lxy = np.sqrt(pcd[0, :] ** 2 + pcd[1, :] ** 2)
        theta = np.arctan2(lxy, pcd[2, :])
        phi = np.minimum(theta, theta_max) ** 2
        p_radial = 1 + phi * (k1 + phi * (k2 + phi * (k3 + phi * k4)))
        theta_s = p_radial * theta
        f_radial = np.choose(lxy > 0, (0, theta_s / lxy))
        ixd = f_radial * pcd[0, :]
        iyd = f_radial * pcd[1, :]

        ix = ixd * fx - 0.5 + mx - alpha * iyd * fx
        iy = iyd * fy - 0.5 + my

    else:
        raise RuntimeError('Unknown intrinsic model ID %d' %
                           invIntrinsic2D['modelID'])

    return np.vstack([ix, iy])


def rectify(invIntrinsic, modelID, image):
    fx, fy, mx, my, alpha, k1, k2, k3, k4, k5 = invIntrinsic[:10]

    x, y = np.meshgrid(np.arange(image.shape[1]), np.arange(image.shape[0]))

    # transformation of pixel coordinates to normalized camera coordinate \
    # system
    ud = ((x + 0.5) - mx) / fx
    vd = ((y + 0.5) - my) / fy

    if modelID in [0, 1]:  # Bouguet model
        # apply distortions
        rd2 = (ud * ud) + (vd * vd)
        radial = 1.0 + (rd2 * (k1 + (rd2 * (k2 + (rd2 * k5)))))
        h = 2 * ud * vd
        tangx = (k3 * h) + (k4 * (rd2 + (2 * ud * ud)))
        tangy = (k3 * (rd2 + (2 * vd * vd))) + (k4 * h)
        distorted_x = (ud * radial) + tangx
        distorted_y = (vd * radial) + tangy

    elif modelID in [2, 3]:  # fish eye model
        fx, fy, mx, my, alpha, k1, k2, k3, k4, theta_max = invIntrinsic[:10]

        lxy = np.sqrt(ud ** 2 + vd ** 2)
        theta = np.arctan2(lxy, 0.5)
        phi = np.minimum(theta, theta_max) ** 2
        p_radial = 1 + phi * (k1 + phi * (k2 + phi * (k3 + phi * k4)))
        theta_s = p_radial * theta
        f_radial = np.choose(lxy > 0, (0, theta_s / lxy))
        distorted_x = f_radial * ud
        distorted_y = f_radial * vd

    else:
        raise RuntimeError('wrong modelID2D')

    # convert back to pixel coordinates for rectification
    ix = ((fx * (distorted_x + (alpha * distorted_y))) + mx) - 0.5
    iy = ((fy * distorted_y) + my) - 0.5

    im_rect = np.empty_like(image)
    if len(image.shape) == 2:
        im_rect = ndimage.map_coordinates(
            image, [iy.ravel(), ix.ravel()], order=3, mode='constant', cval=0
        ).reshape(image.shape[:2])
    else:
        for i in range(image.shape[2]):
            im_rect[:, :, i] = ndimage.map_coordinates(
                image[:, :, i],
                [iy.ravel(), ix.ravel()],
                order=3,
                mode='constant',
                cval=0,
            ).reshape(image.shape[:2])

    return im_rect


class Registration2D3D:
    def __init__(self, depth_prefix, rgb_prefix, use_cloud=True):
        self.use_cloud = use_cloud
        self.point_cloud = None
        self.depth_img = None
        self.rgb_img = None
        self.depth_info = None
        self.rgb_info = None
        self.cv_bridge = CvBridge()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        if use_cloud:
            self.cloud_sub = rospy.Subscriber(
                depth_prefix + "/cloud",
                PointCloud2,
                self.cloud_cb,
                queue_size=1,
            )
        else:
            self.depth_img_sub = rospy.Subscriber(
                depth_prefix + "/distance",
                Image,
                self.depth_image_cb,
                queue_size=1,
            )
            self.depth_info_sub = rospy.Subscriber(
                depth_prefix + "/intrinsics",
                Intrinsics,
                self.depth_info_cb,
                queue_size=1,
            )
        self.rgb_img_sub = rospy.Subscriber(
            rgb_prefix + "/rgb_image/compressed",
            CompressedImage,
            self.rgb_image_cb,
            queue_size=1,
        )
        self.rgb_info_sub = rospy.Subscriber(
            rgb_prefix + "/rgb_info",
            RGBInfo,
            self.rgb_info_cb,
            queue_size=1,
        )
        self.colored_pcl_pub = rospy.Publisher(
            depth_prefix + "/colored_cloud", PointCloud2, queue_size=1
        )

    def cloud_cb(self, msg):
        if rospy.get_time() - msg.header.stamp.to_sec() > 0.1:
            return
        point_cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg).T
        self.pub_colored_pc(point_cloud, msg.header, msg.height, msg.width)

    def depth_image_cb(self, msg):
        if rospy.get_time() - msg.header.stamp.to_sec() > 0.1 or \
                self.depth_info is None:
            return
        point_cloud = self.depth_to_point_cloud(msg, self.depth_info)
        self.pub_colored_pc(point_cloud, msg.header, msg.height, msg.width)

    def rgb_image_cb(self, msg):
        if rospy.get_time() - msg.header.stamp.to_sec() > 0.05:
            return
        self.rgb_img = msg

    def depth_info_cb(self, msg):
        self.depth_info = msg

    def rgb_info_cb(self, msg):
        self.rgb_info = msg

    def depth_to_point_cloud(self, depth_img, depth_info):
        dis = self.cv_bridge.imgmsg_to_cv2(depth_img, depth_img.encoding)
        modelID3D = depth_info.model_id
        intrinsics3D = depth_info.model_parameters
        # Following snippet is obtained from official documentation for o3r.
        #
        # Link for code:
        # https://github.com/ifm/documentation/blob/d844314f891a48b455ba16e1b3f333bee59bc2b3/SoftwareInterfaces/Toolbox/Registration2d3d/2D-3D_registration.py

        # calculate 3D unit vectors corresponding to each pixel
        # of depth camera
        ux, uy, uz = intrinsic_projection(
            modelID3D, intrinsics3D, *dis.shape[::-1])
        # multiply unit vectors by depth of corresponding pixel
        x = (ux * dis).flatten()
        y = (uy * dis).flatten()
        z = (uz * dis).flatten()
        valid = dis.flatten() > 0.05
        for i, pt_valid in enumerate(valid):
            if not pt_valid:
                x[i] = y[i] = z[i] = 0.0
        # Restructure point cloud as sequence of points
        point_cloud = np.stack((x, y, z), axis=0)
        return point_cloud

    def pub_colored_pc(self, point_cloud, header, height, width):
        if self.rgb_img is None or self.rgb_info is None:
            return

        jpg = self.cv_bridge.compressed_imgmsg_to_cv2(self.rgb_img, 'rgb8')
        modelID2D = self.rgb_info.intrinsics.model_id
        invIntrinsic2D = self.rgb_info.inverse_intrinsics.model_parameters

        # try:
        depth_tf = self.tf2_buffer.lookup_transform(
            "base_link",
            header.frame_id,
            rospy.Time.now(),
            rospy.Duration(0.5)
        )
        # TODO: check if same transform
        rgb_tf = self.tf2_buffer.lookup_transform(
            "base_link",
            self.rgb_img.header.frame_id,
            rospy.Time.now(),
            rospy.Duration(0.5)
        )
        # except (tf2_ros.TimeoutException,
        #         tf2_ros.TransformException,
        #         tf2_ros.ExtrapolationException):
        #     rospy.logwarn_throttle(1, "Receiving no transforms")
        #     return

        depth_tf = Transformation.create(depth_tf).to_matrix()
        rgb_tf_inv = Transformation.create(rgb_tf).inverse().to_matrix()

        # Transform from optical coordinate system
        # to user coordinate system
        points = point_cloud.T @ depth_tf[:3, :3].T + depth_tf[:3, 3].T

        # convert to points in optics space
        # reverse internal Transform Rotation
        pcd_o2 = points @ rgb_tf_inv[:3, :3].T + rgb_tf_inv[:3, 3].T

        # Calculate 2D pixel coordinates for each 3D pixel
        pixels = np.round(
            inverse_intrinsic_projection(pcd_o2.T, invIntrinsic2D, modelID2D)
        )

        # Get 2D jpg-color for each 3D-pixel
        # shape is Nx3 (for open3d)
        self.colors = np.zeros((len(pixels[0]), 3))
        for i in range(len(self.colors)):
            idxX = int(pixels[1][i])
            idxY = int(pixels[0][i])
            # Ignore invalid values
            if idxY > 1279 or idxX > 799 or idxY < 0 or idxX < 0:
                self.colors[i, 0] = 126
                self.colors[i, 1] = 126
                self.colors[i, 2] = 126
            else:
                self.colors[i, 0] = jpg.data[idxX, idxY, 0]
                self.colors[i, 1] = jpg.data[idxX, idxY, 1]
                self.colors[i, 2] = jpg.data[idxX, idxY, 2]

        stacked_points = np.column_stack((point_cloud.T, self.colors / 255))

        cloud_type = 'xyzrgb'
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = stacked_points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate(cloud_type)]

        colored_pcl = PointCloud2()
        colored_pcl.header.frame_id = header.frame_id
        colored_pcl.header.stamp = rospy.Time.now()
        colored_pcl.height = height
        colored_pcl.width = width
        colored_pcl.is_bigendian = False
        colored_pcl.is_dense = True
        colored_pcl.fields = fields
        colored_pcl.point_step = (len(cloud_type) * itemsize)
        colored_pcl.row_step = (len(cloud_type) * itemsize * height * width)
        colored_pcl.data = data
        self.colored_pcl_pub.publish(colored_pcl)


if __name__ == "__main__":
    rospy.init_node("registration_2d_3d")
    pcl_coloring_top = Registration2D3D(
        "/ifm3d_ros_driver/top_depth_camera",
        "/ifm3d_ros_driver/top_rgb_camera",
    )
    pcl_coloring_bottom = Registration2D3D(
        "/ifm3d_ros_driver/bottom_depth_camera",
        "/ifm3d_ros_driver/bottom_rgb_camera",
    )
    rospy.spin()
