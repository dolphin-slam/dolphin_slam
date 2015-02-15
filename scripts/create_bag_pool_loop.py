# -*- coding: utf-8 -*-
"""
Created on Wed Sep  3 12:51:24 2014

@author: ballester
"""

#imports
import rosbag
import re
import tf
from PIL import Image as Imagem
import rospy
import Image as Image_py
from sensor_msgs.msg import Image, CameraInfo, Imu
from underwater_sensor_msgs.msg import DVL
from numpy import array, dot
image = Image()
imu = Imu()
dvl = DVL()

#parameters
imu_filename = '/mnt/Dados/Datasets/pool_loop/csv/fog.csv'
dvl_filename = '/mnt/Dados/Datasets/pool_loop/csv/dvl.csv'
img_filename = '/mnt/Dados/Datasets/pool_loop/csv/images.csv'
img_dir		 = '/mnt/Dados/Datasets/pool_loop/imgs/'
img_list_filename = '/mnt/Dados/Datasets/pool_loop/imgs/img_list.txt'
bag_output   = '/home/lsilveira/Codigos/indigo_ws/src/dolphin_slam/experiments/pool_loop.bag'
dvl_it = 2
imu_it = 4
img_it = 1
linear_covariance = False
angular_covariance = False
imu_orientation = True
dvl_bi = True
dvl_wi = True
dvl_bd = True
write_imu = True
write_dvl = True
write_img = True

#reading data
imu_file = open(imu_filename,'r')
dvl_file = open(dvl_filename,'r')
img_file = open(img_filename,'r')
img_list_file = open(img_list_filename,'r')

#cleaning
dvl_content = dvl_file.readline()
imu_content = imu_file.readline()
img_content = img_file.readline()

with rosbag.Bag(bag_output,'w')	as bagui:		#Opening bagfile

	#seq start
	seq_dvl = 0
	seq_imu = 0
	seq_img = 0
	for j in xrange(1100):

		
		#IMU SENSOR
		for i in xrange(imu_it):
			seq_imu += 1
			imu_content = imu_file.readline()
			imu_content = re.split(r'\t+', imu_content.rstrip('\t'))
			yaw = imu_content[8]
			roll = imu_content[9]
			pitch = imu_content[10]
		
			#configuring header:
			imu.header.seq = seq_imu
			imu.header.stamp = rospy.Time.from_sec(float(imu_content[4])*60.0 + float(imu_content[5]) + float(imu_content[6]) * (10**-3))
			imu.header.frame_id = '/base'
		
			#configuring information from the dataset
			if imu_orientation:
				imu_quat = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))
				imu.orientation.x = imu_quat[0]
				imu.orientation.y = imu_quat[1]
				imu.orientation.z = imu_quat[2]
				imu.orientation.w = imu_quat[3] 
	
			if linear_covariance:
				imu.linear_acceleration_covariance.x = 1
				imu.linear_acceleration_covariance.y = 1
				imu.linear_acceleration_covarianze.z = 1	
			
			if write_imu:
				bagui.write('/imu', imu, imu.header.stamp)
			
		
			
			
		for i in xrange(dvl_it):
			seq_dvl += 1
			dvl_content = dvl_file.readline()
			dvl_content = re.split(r'\t+', dvl_content.rstrip('\t'))
		
			#configuring header:
			dvl.header.seq = seq_dvl
			dvl.header.stamp = rospy.Time.from_sec(float(dvl_content[4])*60.0 + float(dvl_content[5]) + float(dvl_content[6]) * (10**-3))		
			dvl.header.frame_id = '/base'
		
			#configuring the information from the dataset
			if dvl_bi:
				dvl.bi_x_axis = float(dvl_content[21])
				dvl.bi_y_axis = float(dvl_content[22])
				dvl.bi_z_axis = float(dvl_content[23])
				dvl.bi_error = float(dvl_content[24])
				dvl.bi_status = str(dvl_content[25])

			if dvl_wi:
				dvl.wi_x_axis = float(dvl_content[16])
				dvl.wi_y_axis = float(dvl_content[17])
				dvl.wi_z_axis = float(dvl_content[18])
				dvl.wi_error = float(dvl_content[19])
				dvl.wi_status = str(dvl_content[20])
		
			if dvl_bd:
				dvl.bd_east = float(dvl_content[43])
				dvl.bd_north = float(dvl_content[44])
				dvl.bd_upwards = float(dvl_content[45])
				dvl.bd_range = float(dvl_content[46])
				#dvl.bd_time = 
			
			if write_dvl:
				bagui.write('/dvl', dvl, dvl.header.stamp)


		for i in xrange(img_it):
			seq_img += 1
	
			#reading image info
			img_content = img_file.readline()
			img_content = re.split(r'\t+', img_content.rstrip('\t'))
		
			#reading image names
			img_names = img_list_file.readline()
			img_names = img_names[0:30]
		
			#configuring header:
			image.header.seq = seq_img
			image.header.stamp = rospy.Time.from_sec(float(img_content[4])*60.0 + float(img_content[5]) + float(img_content[6]) * (10**-3))
			image.header.frame_id = '/base'
		
			#configuring the information from the dataset
			image.encoding = "rgb8"

			img_data = Imagem.open(img_dir + img_names)
			width, height = img_data.size
			image.width = width
			image.step = 3*width
			image.height = height
			image.data = img_data.tostring()
		
			if write_img:
				bagui.write('/image', image, image.header.stamp)


print 'All done, output bag is: ', bag_output
img_file.close()
img_list_file.close()
imu_file.close()
dvl_file.close()












