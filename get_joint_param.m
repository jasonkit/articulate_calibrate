function joint_param = get_joint_param(theta0, psy)
    
    if (nargin == 1)
        psy = 0;
    end
   
    psy = pi*psy/180;

    joint_param = {};

    H_cam_odo = [   0.998617  -0.0296731  0.0434073   2.09099; ...
                  -0.0520077   -0.678925   0.732364  0.925292; ...
                  0.00773874   -0.733608  -0.679529  0.848966; ...
                           0           0          0         1];

    [U S V] = svd(H_cam_odo(1:3,1:3));
    R_cam_odo = U*V';

    theta = theta0;
    joint_param.theta0 = theta;
    joint_param.R_theta = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

    if psy ~= 0
        v = [0;0;1];
        v = v/norm(v);
        joint_param.R_door    = construct_orientation_matrix(v, [1;0;0]);
        joint_param.R_mirror2  = construct_orientation_matrix([sin(psy);0;cos(psy)], [1;0;0]); 
    else
        joint_param.R_door    = construct_orientation_matrix([0.0230665;-0.0251056;0.999419], [1;0;0]);
        joint_param.R_mirror2  = construct_orientation_matrix([0.141583;0.0015661;0.989925], [1;0;0]); 
    end
    joint_param.R_mirror = joint_param.R_door'*joint_param.R_mirror2;
    
    joint_param.R_cam     = joint_param.R_mirror2'*R_cam_odo;
    joint_param.R_cam2    = joint_param.R_mirror*joint_param.R_cam;

    joint_param.t_o2d = [ 2.303; 0.767;  0.849];
    joint_param.t_d2m = [-0.135;     0;      0]; % 1D translation along x-axis only!
    joint_param.t_m2c = [ 0.158; 0.067; -0.019];

    joint_param.R_ref = R_cam_odo;
    joint_param.t_ref = joint_param.R_mirror2*joint_param.t_m2c + joint_param.R_door*joint_param.t_d2m + joint_param.t_o2d;
end

function euler = YPR(R)
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    euler = [x,y,z];
end
