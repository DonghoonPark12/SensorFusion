import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from kitti_foundation import Kitti, Kitti_util

velo_path='/media/deepcv/Samsung_T3/dataset/MV3D/data/raw/kitti/2011_09_26/2011_09_26_drive_0005_sync/' \
          'velodyne_points/data'

velo = Kitti(frame=89, velo_path=velo_path)
frame = velo.velo_file

print(frame.shape)


#read left camera image
image_type = 'color'  # 'gray' or 'color' image
mode = '00' if image_type == 'gray' else '02'  # image_00 = 'graye image' , image_02 = 'color image'
image_path = '/media/deepcv/Samsung_T3/dataset/MV3D/data/raw/kitti/2011_09_26/2011_09_26_drive_0005_sync/'\
             'image_' + mode + '/data'

# print frame 89 camera image
camera = Kitti(frame=89,camera_path=image_path)
image = camera.camera_file
cv2.imshow('image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
#plt.imshow(image)
print(image.shape)


def print_projection_cv2(points, color, image):
    """ project converted velodyne points into camera image """

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for i in range(points.shape[1]):
        cv2.circle(hsv_image, (np.int32(points[0][i]), np.int32(points[1][i])), 2, (int(color[i]), 255, 255), -1)

    return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)


def print_projection_plt(points, color, image):
    """ project converted velodyne points into camera image """

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for i in range(points.shape[1]):
        cv2.circle(hsv_image, (np.int32(points[0][i]), np.int32(points[1][i])), 2, (int(color[i]), 255, 255), -1)

    return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)

""" save one frame about projecting velodyne points into camera image """
v_fov, h_fov = (-24.9, 2.0), (-90, 90)

v2c_filepath = '/media/deepcv/Samsung_T3/dataset/MV3D/data/raw/kitti/2011_09_26/calib_velo_to_cam.txt'
c2c_filepath = '/media/deepcv/Samsung_T3/dataset/MV3D/data/raw/kitti/2011_09_26/calib_cam_to_cam.txt'

res = Kitti_util(frame=89, camera_path=image_path, velo_path=velo_path,
                 v2c_path=v2c_filepath, c2c_path=c2c_filepath)

img, pnt, c_ = res.velo_projection_frame(v_fov=v_fov, h_fov=h_fov)

result = print_projection_plt(pnt, c_, img) # pnt : shape(2, 34908), c_ : shape(34908,),

# display result image
#plt.subplots(1,1, figsize=(13,3))
#plt.title("Velodyne points to camera image Result")
cv2.imshow('image', result)
cv2.waitKey(0)
cv2.destroyAllWindows()

temp = Kitti(frame=0, camera_path=image_path)
img = temp.camera_file
size = (img.shape[1], img.shape[0])

""" save result video """
fourcc = cv2.VideoWriter_fourcc(*'XVID')
vid = cv2.VideoWriter('projection.avi', fourcc, 25.0, size)
test = Kitti_util(frame='all', camera_path=image_path, velo_path=velo_path, \
                  v2c_path=v2c_filepath, c2c_path=c2c_filepath)

res = test.velo_projection(v_fov=v_fov, h_fov=h_fov)

for frame, point, cc in res:
    image = print_projection_cv2(point, cc, frame)
    vid.write(image)

print('video saved')
vid.release()