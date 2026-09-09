##040003173287424600001
*
* Compile mechanism for m710id_70_v01
*
begin COMPONENT m710id_70_v01
	no_of_joints		6
	no_of_loops		0
	TCP_link_name		k7
	user_TCP :
0	0	1	1370	
0	-1	0	0.000246	
1	0	0	1630	
0	0	0	1	
	inverse_family	-1
	base_name		k1

	begin LINK k2
		joint_from	k1
		transformation_to_father :
1	0	0	0	
0	1	0	0	
0	0	1	0	
0	0	0	1	
		joint_name	j1
		axis_points	0 0 -1000 , 0 0 1000
		param_index	0
		joint_type	rot
		user_value	0
		range		-185 185
		max_speed	180
		max_acceleration	180
	end;

	begin LINK k3
		joint_from	k2
		transformation_to_father :
1	0	0	-0.000325	
0	1	0	0	
0	0	1	0	
0	0	0	1	
		joint_name	j2
		axis_points	150 -1000 565 , 150 1000 565
		param_index	1
		joint_type	rot
		user_value	0
		range		-75 150
		max_speed	180
		max_acceleration	180
	end;

	begin LINK k4
		joint_from	k3
		transformation_to_father :
1	0	0	0.002102	
0	1	0	0	
0	0	1	-0.000136	
0	0	0	1	
		joint_name	j3
		axis_points	150 1000 1460 , 150 -1000 1460
		param_index	2
		follows j2 by 1
		joint_type	rot
		user_value	0
		dependent range on joint	j2
		extreme		-75	273 ;
		extreme		-75	-5.5 ;
		extreme		20	-100.502 ;
		extreme		149	-167.580005 ;
		extreme		149.789998	-168 ;
		extreme		150	-168 ;
		extreme		150	55 ;
		extreme		-68	273
		max_speed	180
		max_acceleration	180
	end;

	begin LINK k5
		joint_from	k4
		transformation_to_father :
1	0	0	0.000473	
0	1	0	-0.001722	
0	0	1	0	
0	0	0	1	
		joint_name	j4
		axis_points	2000 0 1630 , -1000 0 1630
		param_index	3
		joint_type	rot
		user_value	0
		range		-400 400
		max_speed	260
		max_acceleration	260
	end;

	begin LINK k6
		joint_from	k5
		transformation_to_father :
1	0	0	0.000977	
0	1	0	0	
0	0	1	-0.000854	
0	0	0	1	
		joint_name	j5
		axis_points	1195 1000 1630 , 1195 -1000 1630
		param_index	4
		joint_type	rot
		user_value	0
		range		-125 125
		max_speed	260
		max_acceleration	260
	end;

	begin LINK k7
		joint_from	k6
		transformation_to_father :
1	0	0	0	
0	1	0	0	
0	0	1	0	
0	0	0	1	
		joint_name	j6
		axis_points	2000 0 1630 , -1000 0 1630
		param_index	5
		joint_type	rot
		user_value	0
		range		-400 400
		max_speed	370
		max_acceleration	370
	end

end
