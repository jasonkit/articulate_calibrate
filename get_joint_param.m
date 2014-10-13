function joint_param = get_joint_param()

    joint_param = {};

    % mirror rotation -pi/4 to pi/4
    %phi = pi/10;
    phi = 0;
    joint_param.phi0 = phi;
    joint_param.R_phi = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];

    joint_param.R_door    = construct_orientation_matrix([.2;.1;1], [1;0;0]);
    joint_param.R_mirror  = construct_orientation_matrix([.3;.2;1], [0;1;0]);
    joint_param.R_mirror2 = joint_param.R_door*joint_param.R_mirror;
    joint_param.R_cam     = construct_orientation_matrix([1;.1;.2], [0;-1;0]);
    joint_param.R_cam2    = joint_param.R_mirror*joint_param.R_phi*joint_param.R_cam;
    
    joint_param.t_o2d = [10;3;2];
    joint_param.t_d2m = [-4;0;0]; % 1D translation along x-axis only!
    joint_param.t_m2c = [1;0;0];
    %joint_param.t_o2m = joint_param.R_door*joint_param.t_d2m + joint_param.t_o2d;
    %joint_param.t_d2c = joint_param.R_mirror*joint_param.R_phi*joint_param.t_m2c + joint_param.t_d2m;

    joint_param.q_door    = mat2q(joint_param.R_door);
    joint_param.q_mirror  = mat2q(joint_param.R_mirror);
    joint_param.q_mirror2 = mat2q(joint_param.R_mirror2);
    joint_param.q_cam     = mat2q(joint_param.R_cam);
    joint_param.q_cam2    = mat2q(joint_param.R_cam2); 
end
