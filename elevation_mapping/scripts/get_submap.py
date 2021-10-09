#!/usr/bin/env python3

import rospy
from grid_map_msgs.srv import GetGridMap 
import numpy as np


while not rospy.is_shutdown():
	#print('waiting')
	rospy.wait_for_service('/elevation_mapping/get_raw_submap')
	get_raw_submap = rospy.ServiceProxy('/elevation_mapping/get_raw_submap', GetGridMap)
	try: 
		raw_submap = get_raw_submap('map',0.0,0.0,1.5,1.5,[])
		#print('Service successful')
		#print(type(raw_submap))
		
		#file = open("submap.txt","w")
		#file.write(raw_submap)
		
		
	except rospy.ServiceException as exc:
	 	#print("Service did not process request: " + str(exc))
	 	continue;
	
	data = raw_submap.data
	#AttributeError: 'GetGridMapResponse' object has no attribute 'data'

	#print(raw_submap)
	#print(raw_submap.shape)
	
	#try:
#		np.savetxt("height_map.csv", raw_submap, delimiter=",")
#	except:
		#print('not good')


