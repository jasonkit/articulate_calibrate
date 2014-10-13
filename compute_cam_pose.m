function [R,t] = compute_cam_pose(joint_param, theta, phi) 
    R1 = joint_param.R_door;
    R2 = vrrotvec2mat([0 0 1 theta]);
    R3 = joint_param.R_mirror;
    R4 = vrrotvec2mat([0 0 1 phi]);
    R5 = joint_param.R_cam;

    R = R1*R2*R3*R4*R5;
    t = R1*R2*R3*R4*joint_param.t_m2c + R1*R2*joint_param.t_d2m + joint_param.t_o2d;
end
