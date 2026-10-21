limits=robot.joint_limits;zero=robot.named_link_frames(np.zeros(6))
base_inverse=np.linalg.inv(robot.base_transform)
origin={n:(base_inverse@zero[n])[:3,3] for n in ('J1_link','J2_link','J3_link','J4_link','J5_link','J6_link','flange')}
shoulder_offset=origin['J2_link'][0]-origin['J1_link'][0]
upper=origin['J3_link'][2]-origin['J2_link'][2]
elbow_offset=origin['J4_link'][2]-origin['J3_link'][2]
forearm=origin['J5_link'][0]-origin['J4_link'][0]
flange_from_wrist=np.linalg.inv(zero['flange'])@zero['J5_link']
assert np.allclose([shoulder_offset,upper,elbow_offset,forearm],[.15,.895,.17,1.045],atol=1e-12,rtol=0)
assert np.allclose(flange_from_wrist[:3,3],[-.175,0,0],atol=1e-12,rtol=0)
assert np.allclose(origin['J5_link'],origin['J6_link'],atol=1e-12,rtol=0)
lower=math.hypot(forearm,elbow_offset);offset_angle=math.atan2(forearm,elbow_offset)
def aliases(angle,j):
    # Full coverage for this model's literal finite bounds (all within +/- 400 deg).
    return [angle+2*math.pi*k for k in range(-3,4) if limits[j,0]-1e-12<=angle+2*math.pi*k<=limits[j,1]+1e-12]
def audit(req):
    goal=np.asarray(req['goal_pose'])
    flange=goal@np.linalg.inv(np.asarray(req['flange_from_task_tcp']))
    wrist_world=(flange@flange_from_wrist)[:3,3]
    wrist=(base_inverse@np.r_[wrist_world,1.])[:3]
    x,y,z=wrist-origin['J1_link'];radius=math.hypot(x,y)
    branches=[]
    for shoulder_sign in (1,-1):
        q1=math.atan2(y,x)+(0 if shoulder_sign==1 else math.pi)
        X=shoulder_sign*radius-shoulder_offset;Z=z
        cosine=(X*X+Z*Z-upper*upper-lower*lower)/(2*upper*lower)
        if abs(cosine)>1+1e-12:
            branches.append(dict(shoulder_sign=shoulder_sign,outside_unbounded_two_link_reach=True,cosine=cosine));continue
        for elbow_sign in (1,-1):
            bend=elbow_sign*math.acos(max(-1.,min(1.,cosine)))
            q3=offset_angle-bend
            q2=math.atan2(X,Z)-math.atan2(lower*math.sin(bend),upper+lower*math.cos(bend))
            raw=[q1,q2,q3];legal=[aliases(q,j) for j,q in enumerate(raw)]
            representative=[xs[0] if xs else min((q+2*math.pi*k for k in range(-3,4)),key=lambda v:max(limits[j,0]-v,v-limits[j,1],0))
                            for j,(q,xs) in enumerate(zip(raw,legal))]
            q=np.r_[representative,[0.,0.,0.]]
            actual=robot.named_link_frames(q)['J5_link'][:3,3]
            error=float(np.linalg.norm(actual-wrist_world))
            assert error<1e-9, (representative,error)
            branches.append(dict(shoulder_sign=shoulder_sign,elbow_sign=elbow_sign,q123_rad=representative,
                q123_deg=np.rad2deg(representative).tolist(),legal_angle_aliases_rad=legal,
                arm_within_limits=all(legal),violating_joints=[f'J{j+1}' for j,xs in enumerate(legal) if not xs],
                production_wrist_fk_error_m=error))
    return dict(stage_id=req['stage_id'],goal_pose=req['goal_pose'],wrist_center_world_m=wrist_world.tolist(),
                branches=branches,legal_arm_branch_count=sum(bool(b.get('arm_within_limits')) for b in branches))
