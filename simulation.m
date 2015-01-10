function ejp = simulation(noise_lv, verbose, theta0, psy)
    if nargin == 0
        noise_lv = 5;
        verbose = true;
        theta0 = 0;
        psy = 0;
    elseif nargin == 1
        verbose = true;
        theta0 = 0;
        psy = 0;
    end

    sim_config = get_simulation_config(noise_lv, 1000, 50, verbose);
    joint_param = get_joint_param(theta0, psy);

    measurements_fix_phi = simulate_door_movement(sim_config, joint_param);
    measurements_fix_theta = simulate_mirror_movement(sim_config, joint_param);
    
    % estimated joint param
    ejp = {};
    R_door_z = estimate_z_axis(measurements_fix_phi);
    ejp.R_door = construct_orientation_matrix(R_door_z, [1;0;0]);

    R_ref = joint_param.R_ref;
    
    R_mirror_z = estimate_z_axis(measurements_fix_theta, R_ref'*ejp.R_door, true);
    R_mirror2_z = ejp.R_door * R_mirror_z; 
    ejp.R_mirror2 = construct_orientation_matrix(R_mirror2_z, [1;0;0]);
    ejp.R_mirror  = ejp.R_door' * ejp.R_mirror2;

    ejp.R_cam     = ejp.R_mirror' * ejp.R_door' * R_ref;
    ejp.R_cam2    = ejp.R_door' * R_ref;
   
    ejp.R_theta = eye(3);
    ejp.theta0 = 0;

    if (joint_param.theta0 ~= 0) 
        is_assume_common_inital_phi = false;

        if (is_assume_common_inital_phi)
            R_ref2 = measurements_fix_theta(1).R;
            ejp.R_theta = ejp.R_door' * R_ref2 * R_ref' * ejp.R_door;
            ejp.theta0 = atan2(ejp.R_theta(2,1), ejp.R_theta(1,1));    
        else
            [ejp.R_theta, ejp.theta0] = estimate_theta0(measurements_fix_theta, ejp.R_door, ejp.R_mirror);
        end
    end

    [theta_m, theta_err] = estimate_theta(ejp, measurements_fix_phi);
    [phi_m, phi_err] = estimate_phi(ejp, measurements_fix_theta);

    ejp = estimate_translation(joint_param.t_o2d(3), ejp, measurements_fix_phi, measurements_fix_theta, theta_m, phi_m);
    ejp.t_ref = ejp.R_mirror2*ejp.t_m2c + ejp.R_door*ejp.t_d2m + ejp.t_o2d;

    if (verbose)
        print_err_report(joint_param, ejp, theta_err, phi_err);
    end
end

function measurements = simulate_door_movement(sim_config, joint_param)
    % door rotation 0 to -70 deg
    theta = rand(1,sim_config.npose)*pi*(70.0/180.0);
    theta = [0 theta];

    measurements = [];
    
    for i=1:sim_config.npose
        measurement = get_measurement(joint_param, sim_config, theta(i), 0);
        if (isempty(measurement) == false)
            measurements = [measurements, measurement];
        end
    end
end

function measurements = simulate_mirror_movement(sim_config, joint_param)
    % mirror rotation 0 to 45 deg
    phi = rand(1,sim_config.npose)*pi/4;
    phi = [0 phi];

    measurements = [];
    
    for i=1:sim_config.npose
        measurement = get_measurement(joint_param, sim_config, joint_param.theta0, phi(i));
        if (isempty(measurement) == false)
            measurements = [measurements, measurement];
        end
    end
end

% Given R_i = R_a*R_z(i)*R_b
% find 3rd column of R_a
function a = estimate_z_axis(measurements, R_k, isTranspose)
    if nargin == 1
        R_k = eye(3);
        isTranspose = false;
    end
    
    npose = size(measurements,2);
    A = [];
    for i=1:npose
        if i == 1;
            R_i = measurements(1).R*R_k;
            R_j = measurements(npose).R*R_k;
        else
            R_i = measurements(i).R*R_k;
            R_j = measurements(i-1).R*R_k;
        end

        if isTranspose == false
            A = [A; R_i*R_j' - eye(3)];
        else
            A = [A; R_i'*R_j - eye(3)];
        end
    end
    
    [U S V] = svd(A);
    a  = V(:,3);
    if (a(3) < 0)
        a = -a;
    end

    a = a/norm(a);
end

function [angles, err] = estimate_rotation_angles(measurements, R1, R2, idx)
    n = size(measurements,2);
    angles = [];
    err = [];
    for i=1:n
        Rz = R1'*measurements(i).R*R2';
        a = atan2(Rz(2,1), Rz(1,1));
        angles = [angles, a];
        err = [err, abs(a-measurements(i).angle_truth(idx))];
    end
end

function [theta, err]  = estimate_theta(jp, measurements)
    [theta, err] = estimate_rotation_angles(measurements, jp.R_door, jp.R_cam2, 1);
end

function [phi, err]  = estimate_phi(jp, measurements)
    R1 = jp.R_door*jp.R_theta*jp.R_mirror;
    [phi, err] = estimate_rotation_angles(measurements, R1, jp.R_cam, 2);
end

function ejp = estimate_translation(h, ejp, measurements_fix_phi, measurements_fix_theta, theta, phi)
    n = size(measurements_fix_phi,2);
    m = size(measurements_fix_theta,2);

    A = [];
    b = [];

    I = eye(3);
    I = I(:,1:2);
    for i=1:n
        R_theta = [cos(theta(i)) -sin(theta(i)) 0; sin(theta(i)) cos(theta(i)) 0; 0 0 1];
        C = ejp.R_door*R_theta;
        D = C*ejp.R_mirror;
       
        A = [A; D, C(:,1), I];
        b = [b; measurements_fix_phi(i).t - [0;0;h]];
    end
    
    for i=1:m
        R_phi = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
        C = ejp.R_door*ejp.R_theta;
        D = C*ejp.R_mirror*R_phi;
        
        A = [A; D, C(:,1), I]; 
        b = [b; measurements_fix_theta(i).t - [0;0;h]];
    end
    % m2c(x,y,z) , d2m(x) o2d(x,y)

    t = pinv(A)*b;
    ejp.t_m2c = t(1:3);
    ejp.t_d2m = [t(4);0;0];
 
    ejp.t_o2d = [t(5:6);h];

end

function [R_theta, theta0] = estimate_theta0(measurements, R_door, R_mirror)
    z = estimate_z_axis(measurements);
    v = R_door' * z;

    m = R_mirror';

    A1 = [m(1,3) m(2,3); m(1,1) m(2,1); m(1,2) m(2,2)];
    b1 = [1-m(3,3)*v(3); -m(3,1)*v(3); -m(3,2)*v(3)];
    r1 = pinv(A1)*b1;
    
    A2 = [m(1,3) m(2,3) 0 0; ...
         0 0 m(1,3) m(2,3); ...
         v(1) 0 v(2) 0; ...
         0 v(1) 0 v(2)];

    b2 = [-m(3,3)*v(1); -m(3,3)*v(2); -v(3)*r1(1); -v(3)*r1(2)];

    p = pinv(A2)*b2;
    q = null(A2);
    q = q/norm(q);
    p = p - (p'*q)*q;
    
    a = q(1)*q(2) + q(3)*q(4);
    b = p(1)*q(2) + p(2)*q(1) + p(3)*q(4) + p(4)*q(3);
    c = p(1)*p(2) + p(3)*p(4) + r1(1)*r1(2);

    eps = 1e-10;
    if (sum(abs(q) < eps) >= 2) 
        % this handle the case where z-axis of R_door's y component is zero
        % this make q has zero component, hence corresponding part of the p is known
        if (abs(q(1)) <= eps && abs(q(3)) <= eps)
            col1 = [p(1);p(3);r1(1)];
            col2 = cross(v,col1);
            R_theta = [col1 col2 v] * R_mirror';
        elseif (abs(q(2)) <= eps && abs(q(4)) <= eps)
            col2 = [p(2);p(4);r1(2)];
            col1 = cross(col2,v);
            R_theta = [col1 col2 v] * R_mirror';
        elseif (abs(q(1)) <= eps && abs(q(2)) <= eps)
            col2 = [p(2);p(4);r1(2)];
            row1 = [p(1) p(2) v(1)];
            row2 = cross([r1(1) r1(2) v(3)], row1);
            R_theta = [row1; row2; r1(1) r1(2) v(3)] * R_mirror';
        else
            row2 = [p(3) p(4) v(2)];
            row1 = cross(row2, [r1(1) r1(2) v(3)]);
            R_theta = [row1; row2; r1(1) r1(2) v(3)] * R_mirror';
        end
    elseif (b*b - 4*a*c >= 0) 
        % current condition of q already make sure a is not zero
        alpha = [-b + sqrt(b*b-4*a*c); -b - sqrt(b*b-4*a*c)]/(2*a);
        
        r2 = p + alpha(1)*q;
        r3 = p + alpha(2)*q;

        R_theta1 = [r2(1) r2(2) v(1); r2(3) r2(4) v(2); r1(1) r1(2) v(3)] * R_mirror';
        R_theta2 = [r3(1) r3(2) v(1); r3(3) r3(4) v(2); r1(1) r1(2) v(3)] * R_mirror';
        
        if (det(R_theta1) > 0)
            R_theta = R_theta1;
        else
            R_theta = R_theta2;
        end 
    else
        % just in case the there is no real solution, pick the closest one
        alpha = -b/(2*a);
        r = p + alpha*q;
        r(1:2) = sqrt((1-v(1)*v(1))/(r(1)*r(1) + r(2)*r(2)))*r(1:2);
        r(3:4) = sqrt((1-v(2)*v(2))/(r(3)*r(3) + r(4)*r(4)))*r(3:4);

        R_theta = [r(1) r(2) v(1); r(3) r(4) v(2); r1(1) r1(2) v(3)] * R_mirror';
    end
    
    theta0 = atan2((R_theta(2,1)-R_theta(1,2)), R_theta(1,1) + R_theta(2,2));
    R_theta = rotation_matrix_from_angle(theta0);
end

function reconstuctCameraPoses(angles, jp, measurements, is_door)
    n = size(angles,2);
    for i = 1:n
        H = eye(4);
        if (is_door)
            H = construct_transfomation_from_angles(jp, angles(i), 0);
        else
            H = construct_transfomation_from_angles(jp, jp.theta0, angles(i));
        end

        inv(H)*[measurements(i).R measurements(i).t; 0 0 0 1]
    end
end

function print_err_report(jp,ejp, theta_err, phi_err)

    R_door_diff    = axis_diff_in_euler(ejp.R_door, jp.R_door);
    R_mirror_diff  = axis_diff_in_euler(ejp.R_mirror, jp.R_mirror);
    R_mirror2_diff = axis_diff_in_euler(ejp.R_mirror2, jp.R_mirror2);
    R_cam_diff     = axis_diff_in_euler(ejp.R_cam, jp.R_cam);
    R_cam2_diff    = axis_diff_in_euler(ejp.R_cam2, jp.R_cam2);

    R_cam_odo_diff = axis_diff_in_euler(ejp.R_door*ejp.R_mirror*ejp.R_cam, jp.R_ref);

    disp(sprintf('theta0 error (deg)\t\t: %f', abs(ejp.theta0 - jp.theta0)/pi*180));
    disp(sprintf('Average theta error (deg)\t: %f', mean(theta_err/pi*180)));
    disp(sprintf('Average phi error (deg)\t\t: %f', mean(phi_err/pi*180)));
    disp('---'); 
    disp(sprintf('R_door error in ZYX (deg)\t x: %f y: %f z: %f', R_door_diff(1), R_door_diff(2), R_door_diff(3)));
    disp(sprintf('R_mirror error in ZYX (deg)\t x: %f y: %f z: %f', R_mirror_diff(1), R_mirror_diff(2), R_mirror_diff(3)));
    disp(sprintf('R_mirror2 error in ZYX (deg)\t x: %f y: %f z: %f', R_mirror2_diff(1), R_mirror2_diff(2), R_mirror2_diff(3)));
    disp(sprintf('R_cam error in ZYX (deg)\t x: %f y: %f z: %f', R_cam_diff(1), R_cam_diff(2), R_cam_diff(3)));
    disp(sprintf('R_cam2 error in ZYX (deg)\t x: %f y: %f z: %f', R_cam2_diff(1), R_cam2_diff(2), R_cam2_diff(3)));
    disp(sprintf('R_cam_odo error in ZYX (deg)\t x: %f y: %f z: %f', R_cam_odo_diff(1), R_cam_odo_diff(2), R_cam_odo_diff(3)));
    disp('---'); 
    disp(sprintf('t_o2d error\t x: %f y: %f z: %f', ejp.t_o2d(1) - jp.t_o2d(1), ejp.t_o2d(2) - jp.t_o2d(2), ejp.t_o2d(3) - jp.t_o2d(3)));
    disp(sprintf('t_d2m error\t x: %f', ejp.t_d2m(1) - jp.t_d2m(1)));
    disp(sprintf('t_m2c error\t x: %f y: %f z: %f', ejp.t_m2c(1) - jp.t_m2c(1), ejp.t_m2c(2) - jp.t_m2c(2), ejp.t_m2c(3) - jp.t_m2c(3))); 
    disp(sprintf('t_ref error\t x: %f y: %f z: %f', ejp.t_ref(1) - jp.t_ref(1), ejp.t_ref(2) - jp.t_ref(2), ejp.t_ref(3) - jp.t_ref(3))); 
    disp('---'); 
    disp(sprintf('t_o2d %% error\t x: %f y: %f z: %f', 100*(ejp.t_o2d(1) - jp.t_o2d(1))/jp.t_o2d(1), 100*(ejp.t_o2d(2) - jp.t_o2d(2))/jp.t_o2d(2), 100*(ejp.t_o2d(3) - jp.t_o2d(3))/jp.t_o2d(3)));
    disp(sprintf('t_d2m %% error\t x: %f', 100*(ejp.t_d2m(1) - jp.t_d2m(1))/jp.t_d2m(1)));
    disp(sprintf('t_m2c %% error\t x: %f y: %f z: %f', 100*(ejp.t_m2c(1) - jp.t_m2c(1))/jp.t_m2c(1), 100*(ejp.t_m2c(2) - jp.t_m2c(2))/jp.t_m2c(2), 100*(ejp.t_m2c(3) - jp.t_m2c(3))/jp.t_m2c(3)));
    disp(sprintf('t_ref %% error\t x: %f y: %f z: %f', 100*(ejp.t_ref(1) - jp.t_ref(1))/jp.t_ref(1), 100*(ejp.t_ref(2) - jp.t_ref(2))/jp.t_ref(2), 100*(ejp.t_ref(3) - jp.t_ref(3))/jp.t_ref(3)));
end

