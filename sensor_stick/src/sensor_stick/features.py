import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized

# Define a function to compute color histogram features  
def color_hist(ch_h, ch_s, ch_v, nbins=32, bins_range=(0, 256)):
    
    # Compute the histogram of the HSV channels separately
    h_hist = np.histogram(ch_h, bins=nbins, range=bins_range)
    s_hist = np.histogram(ch_s, bins=nbins, range=bins_range)
    v_hist = np.histogram(ch_v, bins=nbins, range=bins_range)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
    # Normalize the result
    norm_features = hist_features / np.sum(hist_features)
    # Return the feature vector
    return norm_features
    
def compute_color_histograms(cloud, nbins=32, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    # TODO: Concatenate and normalize the histograms
    
    normed_features = color_hist(channel_1_vals, channel_2_vals, channel_3_vals, nbins = nbins) 
    return normed_features 


def compute_normal_histograms(normal_cloud, nbins=32):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    # TODO: Concatenate and normalize the histograms

    normed_features = color_hist(norm_x_vals, norm_y_vals, norm_z_vals, nbins=nbins) 

    return normed_features
