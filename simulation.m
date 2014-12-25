function ejp = simulation(noise_lv, verbose)
    if nargin == 0
        noise_lv = 5;
        verbose = true;
    elseif nargin == 1
        verbose = true;
    end

    sim_config = get_simulation_config(noise_lv, 1000, 50, verbose);
    joint_param = get_joint_param();

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

    % estimated joint param
    ejp = {};

    ejp.R_door = estimate_R_door(measurements);

    [R_mirror2_observed, measurements_fix_theta] = simulation_mirror(sim_config, ejp.R_door);
    R_mirror_observed = ejp.R_door'*R_mirror2_observed;

    [ejp.R_mirror, ejp.R_mirror2, ejp.R_cam, ejp.R_cam2, ejp.R_theta] = estimate_R_mirror_and_cam(measurements, ejp.R_door, R_mirror_observed, measurements(1).R, measurements_fix_theta(1).R, false);

    ejp.theta0 = atan2(ejp.R_theta(2,1), ejp.R_theta(1,1));

    [theta_m, theta_err] = estimate_theta(ejp, measurements);
    [phi_m, phi_err] = estimate_phi(ejp, measurements_fix_theta);

    ejp = estimate_translation(joint_param.t_o2d(3), ejp, measurements, measurements_fix_theta, theta_m, phi_m);
    ejp.t_ref = ejp.R_mirror2*ejp.t_m2c + ejp.R_door*ejp.t_d2m + ejp.t_o2d;

    if (verbose)
        print_err_report(joint_param, ejp, theta_err, phi_err);
    end
end

function [R_mirror2_estimated, measurements] = simulation_mirror(sim_config, R_door_estimated)
    joint_param = get_joint_param();

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
    
    R_mirror2_estimated = estimate_observed_R_mirror2(measurements);
end

% actually this is not estimating rotational axis!
% it is finding the 3rd column(z-axis) of R_mirror2 or R_door from measurements
% Given R_i = R_a*R_z(i)*R_b
% find 3rd column of R_a
function a = estimate_z_axis_1(measurements)
    npose = size(measurements,2);
    A = [];
    for i=1:npose
        if i == 1;
            A = [A; measurements(1).R*measurements(npose).R' - eye(3)];
        else
            A = [A; measurements(i).R*measurements(i-1).R' - eye(3)];
        end
    end
    
    [U S V] = svd(A);
    a  = V(:,3);
    if (a(1) < 0)
        a = -a;
    end

    a = a/norm(a);
end

function R_door = estimate_R_door(measurements)
    a = estimate_z_axis_1(measurements);
    R_door = construct_orientation_matrix(a, [1;0;0]);
end

function R_mirror2 = estimate_observed_R_mirror2(measurements)
    a = estimate_z_axis_1(measurements);
    R_mirror2 = construct_orientation_matrix(a, [1;0;0]);
end

% given R_a and R_i = R_a*R_z(i)*R_b
% find 3rd row of R_b
function a = estimate_z_axis_2(measurements, R)
    
    npose = size(measurements,2);
    A = zeros(3,3); 
    for i=1:npose
        A = A + R'*measurements(i).R;
    end
    
    [U S V] = svd(A);
    A = U*diag([1,1,det(U)*det(V)])*V';
    
    a = A(3,:)';
    a = a/norm(a);
end


function [R_mirror, R_mirror2, R_cam, R_cam2, R_theta] = estimate_R_mirror_and_cam(measurements, R_door, R_mirror_observed, R_ref, R_ref2, is_ignore_theta0)

    R_theta = eye(3);

    if is_ignore_theta0 == false
        R_theta = R_door'*R_ref2*R_ref'*R_door;
    end

    % reconstruct the real R_mirror and R_mirror2
    R_mirror2 = R_door * R_theta' * R_mirror_observed;
    R_mirror2 = construct_orientation_matrix(R_mirror2(:,3), [1;0;0]);
    R_mirror = R_door' * R_mirror2;
    R_cam = R_mirror' * R_door' * R_ref;
    R_cam2 = R_door' * R_ref;
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

    for i=1:n
        R_theta = [cos(theta(i)) -sin(theta(i)) 0; sin(theta(i)) cos(theta(i)) 0; 0 0 1];
        C = ejp.R_door*R_theta;
        D = C*ejp.R_mirror;
        
        A = [A; D(1,1), D(1,2), D(1,3), C(1,1), 1, 0, 0; ...
                D(2,1), D(2,2), D(2,3), C(2,1), 0, 1, 0; ...
                D(3,1), D(3,2), D(3,3), C(3,1), 0, 0, 0];
        b = [b; measurements_fix_phi(i).t - [0;0;h]];
    end
    
    for i=1:m
        R_phi = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
        C = ejp.R_door*ejp.R_theta;
        D = C*ejp.R_mirror*R_phi;
        
        A = [A; D(1,1), D(1,2), D(1,3), C(1,1), 1, 0, 0; ...
                D(2,1), D(2,2), D(2,3), C(2,1), 0, 1, 0; ...
                D(3,1), D(3,2), D(3,3), C(3,1), 0, 0, 0];
        b = [b; measurements_fix_theta(i).t - [0;0;h]];
    end
    % m2c(x,y,z) , d2m(x) o2d(x,y)

    t = pinv(A)*b;
    ejp.t_m2c = t(1:3);
    ejp.t_d2m = [t(4);0;0];
 
    ejp.t_o2d = [t(5:6);h];

end

function sim_config = get_simulation_config(noise_level, npts, npose, verbose);
    sim_config = {};
    sim_config.npts = npts;
    sim_config.npose = npose;
    sim_config.aspect_ratio = 1.6;
    sim_config.noise = noise_level/1280;
    sim_config.clouds = generate_point_cloud(sim_config.npts,20,30)';
    
    if (verbose)
        disp(sprintf('[Simulation Config] Noise Level: %d, #Point: %d #Pose: %d', noise_level, npts, npose));
    end
end

function euler = axis_diff_in_euler(R1, R2)
    R = R1*R2';
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    euler = [x,y,z];
end

function R = ensure_pure_z_rotation(R)
    a = atan2(R(2,1), R(1,1));
    R = [cos(a) -sin(a) 0; ...
         sin(a)  cos(a) 0; ...
             0       0  1]; 
end

function R = rotation_matrix_from_angle(angle)
    R = [cos(angle), -sin(angle), 0; ...
         sin(angle),  cos(angle), 0; ...
                  0,           0, 1];
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

function H = construct_transfomation_from_angles(jp, theta, phi)
    R1 = jp.R_door * rotation_matrix_from_angle(theta);
    R2 = jp.R_mirror * rotation_matrix_from_angle(phi);
    R = R1*R2*jp.R_cam;
    t = R1*R2*jp.t_m2c + R1*jp.t_d2m + jp.t_o2d;
    H = [R t; 0 0 0 1];
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

