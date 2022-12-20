# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Process the point-cloud and prepare it for object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import cv2
import numpy as np
import torch

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

# object detection tools and helper functions
import misc.objdet_tools as tools

G_INIT = {
    'show_pcl': None,
    'show_pcl_initialized': False,
}


# visualize lidar point-cloud
def show_pcl(pcl):

    ####### ID_S1_EX2 START #######
    #######
    print("student task ID_S1_EX2")

    def close(_1, _2, _3):
        G_INIT['show_pcl'].close()
        G_INIT['show_pcl'] = None
        return True

    # step 1 : initialize open3d with key callback and create window
    import time
    import open3d
    if G_INIT['show_pcl'] is None:
        G_INIT['show_pcl'] = open3d.visualization.VisualizerWithKeyCallback()
        G_INIT['show_pcl'].create_window('Lidar Viewer')
        G_INIT['show_pcl'].register_key_action_callback(262, close)
        G_INIT['show_pcl_initialized'] = False

    # step 2 : create instance of open3d point-cloud class
    # step 3 : set points in pcd instance by converting the point-cloud into 3d vectors (using open3d function Vector3dVector)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pcl[:, :3])

    # step 4 : for the first frame, add the pcd instance to visualization using add_geometry; for all other frames, use update_geometry instead
    if not G_INIT['show_pcl_initialized']:
        G_INIT['show_pcl_initialized'] = True
        G_INIT['show_pcl'].add_geometry(pcd)
    else:
        G_INIT['show_pcl'].update_geometry(pcd)

    # step 5 : visualize point cloud and keep window open until right-arrow is pressed (key-code 262)
    G_INIT['show_pcl'].poll_events()
    G_INIT['show_pcl'].update_renderer()
    time.sleep(3)

    #######
    ####### ID_S1_EX2 END #######
    # Below interaction works, but will create another window.
    # vis = open3d.visualization.VisualizerWithKeyCallback()
    # vis.create_window('Lidar Viewer')
    # vis.register_key_action_callback(262, lambda _1, _2, _3: vis.close() is not None)
    # pcd = open3d.geometry.PointCloud()
    # pcd.points = open3d.utility.Vector3dVector(pcl[:, :3])
    # vis.add_geometry(pcd)
    # vis.run()


# visualize range image
def show_range_image(frame, lidar_name):

    ####### ID_S1_EX1 START #######
    #######
    print("student task ID_S1_EX1")

    # step 1 : extract lidar data and range image for the roof-mounted lidar
    # step 2 : extract the range and the intensity channel from the range image
    # step 3 : set values <0 to zero
    import zlib
    img = None
    for image in frame.lasers:
        if image.name == lidar_name:
            img = dataset_pb2.MatrixFloat()
            img.ParseFromString(zlib.decompress(image.ri_return1.range_image_compressed))
            img = np.array(img.data).reshape(img.shape.dims)
            img[img < 0] = 0
            break

    assert img is not None, 'Lidar name not found'
    channel_range = img[:,:, 0]
    channel_intensity = img[:,:, 1]

    # step 4 : map the range channel onto an 8-bit scale and make sure that the full range of values is appropriately considered
    channel_range /= channel_range.max()
    channel_range *= 255.0
    channel_range = np.array(channel_range, dtype=np.uint8)
    # cv2.imwrite('channel_range.png', channel_range) # for test

    # step 5 : map the intensity channel onto an 8-bit scale and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers
    thres_upper = np.percentile(channel_intensity, 99)
    thres_lower = np.percentile(channel_intensity, 1)
    channel_intensity -= thres_lower
    channel_intensity[channel_intensity > thres_upper] = thres_upper
    channel_intensity /= thres_upper
    channel_intensity *= 255.0
    channel_intensity = np.array(channel_intensity, dtype=np.uint8)
    # cv2.imwrite('channel_intensity.png', channel_intensity) # for test

    # step 6 : stack the range and intensity image vertically using np.vstack and convert the result to an unsigned 8-bit integer
    img_range_intensity = np.vstack((channel_range, channel_intensity,))
    cv2.imwrite('img_range_intensity.png', img_range_intensity) # for test
    assert img_range_intensity.shape[0] == img.shape[0] * 2
    assert img_range_intensity.shape[1] == img.shape[1]
    #######
    ####### ID_S1_EX1 END #######

    return img_range_intensity


# create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs):

    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]

    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    ####### ID_S2_EX1 START #######
    #######
    print("student task ID_S2_EX1")
    # For normalization
    thres_intensity_upper = np.percentile(lidar_pcl[:, 3], 99)
    thres_intensity_lower = np.percentile(lidar_pcl[:, 3], 1)
    thres_height_upper = lidar_pcl[:, 2].max()
    thres_height_lower = lidar_pcl[:, 2].min()
    ## step 1 : compute bev-map discretization by dividing x-range by the bev-image height (see configs)
    ## step 2 : create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates
    # step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur
    lidar_pcl_dis = lidar_pcl[:, :]
    lidar_pcl_dis[:, 0] = np.floor(configs.bev_height * (lidar_pcl[:, 0] - configs.lim_x[0]) / (configs.lim_x[1] - configs.lim_x[0]))
    lidar_pcl_dis[:, 1] = np.floor(configs.bev_width * (lidar_pcl[:, 1] - configs.lim_y[0]) / (configs.lim_y[1] - configs.lim_y[0]))

    # step 4 : visualize point-cloud using the function show_pcl from a previous task
    # show_pcl(lidar_pcl_dis)
    #######
    ####### ID_S2_EX1 END #######


    # Compute intensity layer of the BEV map
    ####### ID_S2_EX2 START #######
    #######
    print("student task ID_S2_EX2")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    intensity_map = np.zeros([configs.bev_height, configs.bev_width], dtype=lidar_pcl.dtype)

    # step 2 : re-arrange elements in lidar_pcl_dis by sorting first by x, then y, then -z (use numpy.lexsort)
    lidar_pcl_top = lidar_pcl_dis[:, :]
    indices = np.lexsort([-lidar_pcl_top[:, 2], lidar_pcl_top[:, 1], lidar_pcl_top[:, 0]])
    lidar_pcl_top = lidar_pcl_top[indices, :]

    ## step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    ##          also, store the number of points per x,y-cell in a variable named "counts" for use in the next task
    _, unique_ids, counts = np.unique(lidar_pcl_top[:, :2], axis=0, return_index=True, return_counts=True)
    lidar_pcl_top = lidar_pcl_top[unique_ids, :]

    ## step 4 : assign the intensity value of each unique entry in lidar_top_pcl to the intensity map
    ##          make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible
    ##          also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud
    def normalize(value: float, upper: float, lower: float) -> float:
        return max(0.0, min(1.0, (value - lower) / (upper - lower)))
    def normalize_intensity(value: float) -> float:
        return normalize(value=value, upper=thres_intensity_upper, lower=thres_intensity_lower)

    for pt in lidar_pcl_top:
        intensity_map[int(pt[0]), int(pt[1])] = normalize_intensity(pt[3])

    ## step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    cv2.imwrite('intensity_map.png', intensity_map * 255)
    #######
    ####### ID_S2_EX2 END #######


    # Compute height layer of the BEV map
    ####### ID_S2_EX3 START #######
    #######
    print("student task ID_S2_EX3")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    height_map = np.zeros([configs.bev_height, configs.bev_width], dtype=lidar_pcl.dtype)

    ## step 2 : assign the height value of each unique entry in lidar_top_pcl to the height map
    ##          make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    ##          use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map
    def normalize_height(value: float) -> float:
        return normalize(value=value, upper=thres_height_upper, lower=thres_height_lower)
    for pt in lidar_pcl_top:
        height_map[int(pt[0]), int(pt[1])] = normalize_height(pt[2])

    ## step 3 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    cv2.imwrite('height_map.png', height_map * 255)

    #######
    ####### ID_S2_EX3 END #######

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_dis[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64))
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalizedCounts

    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps


