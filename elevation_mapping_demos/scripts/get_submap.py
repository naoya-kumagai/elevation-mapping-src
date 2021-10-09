#!/usr/bin/env python3

import rospy

while True:
	print('waiting')
	rospy.wait_for_service('/elevation_mapping/get_raw_submap')
	get_raw_submap = rospy.ServiceProxy('get_raw_submap', elevation_mapping.srv.GetGridMap)
	try: 
		resp1 = get_raw_submap('odom',0,0,1,1,[])
	except rospy.ServiceException as exc:
	 	print("Service did not process request: " + str(exc))
	 	continue;
	print('Service successfull')
	 	




