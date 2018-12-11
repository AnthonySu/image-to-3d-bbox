import matplotlib.pyplot as plt
import numpy as np
import cv2
import parseTrackletXML as xmlParser
import os

# Set path for the label directory and image directory
label_dir = '2011_09_26_drive_0009_sync/predict_02/'
image_dir = '2011_09_26_drive_0009_sync/image_02/data/'
calib_dir = '2011_09_26_drive_0009_sync/calib_02/'
dataset = [name.split('.')[0] for name in sorted(os.listdir(label_dir))]

# Export the video:
video_res = 'kitti_3D.avi'
video_writer = None
# Sort all the images so that we could index them:
images = sorted(os.listdir(image_dir))
for f in images:
    image_file = image_dir + f
    calib_file = calib_dir + f.replace('png', 'txt')
    # predi_file = predi_dir + f.replace('png', 'txt')

    # read calibration data
    for line in open(calib_file):
        if 'P2:' in line:
            cam_to_img = line.strip().split(' ')
            cam_to_img = np.asarray([float(number) for number in cam_to_img[1:]])
            cam_to_img = np.reshape(cam_to_img, (3,4))
        
    image = cv2.imread(image_file)
    cars = []




    if video_writer is None:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_writer = cv2.VideoWriter(video_out, fourcc, 25.0, (1242, 375))

    # Draw 3D Bounding Box
    for line in open(predi_file):
        line = line.strip().split(' ')

        dims   = np.asarray([float(number) for number in line[8:11]])
        center = np.asarray([float(number) for number in line[11:14]])
        rot_y  = float(line[3]) + np.arctan(center[0]/center[2])#float(line[14])

        box_3d = []

        for i in [1,-1]:
            for j in [1,-1]:
                for k in [0,1]:
                    point = np.copy(center)
                    point[0] = center[0] + i * dims[1]/2 * np.cos(-rot_y+np.pi/2) + (j*i) * dims[2]/2 * np.cos(-rot_y)
                    point[2] = center[2] + i * dims[1]/2 * np.sin(-rot_y+np.pi/2) + (j*i) * dims[2]/2 * np.sin(-rot_y)                  
                    point[1] = center[1] - k * dims[0]

                    point = np.append(point, 1)
                    point = np.dot(cam_to_img, point)
                    point = point[:2]/point[2]
                    point = point.astype(np.int16)
                    box_3d.append(point)

        for i in xrange(4):
            point_1_ = box_3d[2*i]
            point_2_ = box_3d[2*i+1]
            cv2.line(image, (point_1_[0], point_1_[1]), (point_2_[0], point_2_[1]), (0,255,0), 2)

        for i in xrange(8):
            point_1_ = box_3d[i]
            point_2_ = box_3d[(i+2)%8]
            cv2.line(image, (point_1_[0], point_1_[1]), (point_2_[0], point_2_[1]), (0,255,0), 2)
                
    video_writer.write(np.uint8(image))