import cv2
import numpy as np 
from PIL import Image 
from skimage.color import rgb2gray
from skimage.util import invert
from skimage.morphology import skeletonize
from skimage import measure


def get_image(filename, kind='rgb'):
    '''
    Get and return image 
    '''
    img = Image.open(filename)
    img_numpy = np.array(img)

    if kind == 'gray':
        return rgb2gray(img_numpy)
    else:
        return img_numpy

def get_path(img):
    ''' 
    Get only path from image
    '''
    thresh = .55
    if img.shape > 2:
        raise ValueError("Needs gray scale image")
    binary = img > thresh
    return binary

def get_skeleton(img):
    '''
    Takes in the route image on the node network 
    and returns a skeletonized version 

    Parameters:
        input: grayscale image [0,1]
        output: grayscale image [0,1]
    '''
    # img_thresh = np.where(img > .55, 1, 0)
    img_thresh = img > .55
    img_thresh = invert(img_thresh) + 2
    skeleton = skeletonize(img_thresh) * 1.

    return skeleton


def get_rot_scheme(img):
    '''
    Calculates a measure for the amount of 
    curvature in the image path
    '''
    pass
