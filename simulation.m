function ejp = simulation(noise_lv)
    if nargin == 0
        noise_lv = 5;
    end

    sim_config = get_simulation_config(noise_lv,1000,50);
    joint_param = get_joint_param();

    % door rotation 0 to pi/3
    theta = -rand(1,sim_config.npose)*pi/3;

    measurements = [];
    
    for i=1:sim_config.npose
        measurement = get_measurement(joint_param, sim_config, theta(i), joint_param.phi0);
        if (isempty(measurement) == false)
            measurements = [measurements, measurement];
        end
    end

    % estimated joint param
    ejp = {};

    ejp.R_door = estimate_R_door(measurements);
    [ejp.R_mirror2, ejp.R_cam, measurements_fix_theta] = simulation_mirror(sim_config, ejp.R_door);
    ejp.R_mirror = ejp.R_door'*ejp.R_mirror2;
    [ejp.R_cam2, ejp.R_phi] = estimate_R_cam2(measurements, ejp.R_door, ejp.R_mirror, ejp.R_cam);

    ejp.q_door    = mat2q(ejp.R_door);
    ejp.q_mirror  = mat2q(ejp.R_mirror);
    ejp.q_mirror2 = mat2q(ejp.R_mirror2);
    ejp.q_cam     = mat2q(ejp.R_cam);
    ejp.q_cam2    = mat2q(ejp.R_cam2);

    ejp.phi0 = atan2(ejp.R_phi(2,1), ejp.R_phi(1,1));

    [theta_m, theta_err] = estimate_theta(ejp, measurements);
    [phi_m, phi_err] = estimate_phi(ejp, measurements_fix_theta);

    ejp = estimate_translation(joint_param.t_o2d(3), ejp, measurements, measurements_fix_theta, theta_m, phi_m, joint_param);

    print_err_report(joint_param, ejp, theta_err, phi_err);
end

function [R_mirror2_estimated, R_cam_estimated, measurements] = simulation_mirror(sim_config, R_door_estimated)
    joint_param = get_joint_param();

    % mirror rotation -pi/4 to pi/4
    phi = rand(1,sim_config.npose)*pi/2 - pi/4;

    measurements = [];
    
    for i=1:sim_config.npose
        measurement = get_measurement(joint_param, sim_config, 0, phi(i));
        if (isempty(measurement) == false)
            measurements = [measurements, measurement];
        end
    end
    
    R_mirror2_estimated = estimate_R_mirror(measurements, R_door_estimated);
    R_cam_estimated = estimate_R_cam(measurements, R_mirror2_estimated);
end

function a = estimate_rotation_axis(measurements)
    npose = size(measurements,2);
    a = [0;0;0];

    for i=1:npose
        A = zeros(3,3);
        if i == 1;
            A = measurements(1).R*measurements(npose).R' - eye(3);
        else
            A = measurements(i).R*measurements(i-1).R' - eye(3);
        end
        [U S V] = svd(A);
        a_estimated = V(:,3);
        if (a_estimated(1) < 0)
            a_estimated = -a_estimated;
        end

        a = a + a_estimated;
    end

    a = a/npose;
    a = a/norm(a);
end

function R_mirror = estimate_R_mirror(measurements, R_door)
    a = estimate_rotation_axis(measurements); 
    R_mirror = construct_orientation_matrix(a, R_door(:,2));
end

function R_door = estimate_R_door(measurements)
    a = estimate_rotation_axis(measurements);
    R_door = construct_orientation_matrix(a, [1;0;0]);
end

% R_cam estimation is not trivial, proved by using matlab symbolic evalulation
function R_cam = estimate_R_cam(measurements, R_mirror)
    
    npose = size(measurements,2);
    a = [0;0;0];
    for i=1:npose
        B = R_mirror'*measurements(i).R;
        a = a + B(3,:)';
    end

    % alternate way to get third row of R_cam is simular to estimate_R_door/estimate_R_mirror
    % but simulation showing better by using estimated R_mirror to estimate R_cam

    a = a/npose;
    a = a/norm(a);

    % using only third row of R_cam to reconstruct R_cam seems not trivial
    % this can be proved by matlab symbolic evaluation
    % seems only possible if {R_cam}x = +/- {m'}y

    R_cam = construct_orientation_matrix(a, [0;1;0])';
    R_cam(1:2,:) = sign(a(3))*R_cam(1:2,:);
end

function [R_cam2, R_phi] = estimate_R_cam2(measurements, R_door, R_mirror, R_cam1)
    npose = size(measurements,2);
    a = [0;0;0];

    for i=1:npose
        B = R_door'*measurements(i).R;
        a = a + B(3,:)';
    end
    a = a/npose;
    a = a/norm(a);

    C = R_cam1;
    M = R_mirror;
    A = [C(1,1)*M(3,2)-C(2,1)*M(3,1), C(1,1)*M(3,1)+C(2,1)*M(3,2); ...
         C(1,2)*M(3,2)-C(2,2)*M(3,1), C(1,2)*M(3,1)+C(2,2)*M(3,2)];
    
    b = [a(1) - C(3,1)*M(3,3); ...
         a(2) - C(3,2)*M(3,3)];

    alpha = inv(A)*b;
    alpha = alpha/norm(alpha);
    R_phi = [alpha(2) -alpha(1) 0; alpha(1) alpha(2) 0; 0 0 1];

    R_cam2 = R_mirror*R_phi*R_cam1;
end

function [theta, err]  = estimate_theta(joint_param, measurements)
    n = size(measurements,2);
    theta = [];
    err = [];
    for i=1:n
        Rz = joint_param.R_door'*measurements(i).R*joint_param.R_cam2';
        a = atan2(Rz(2,1), Rz(1,1));
        theta = [theta, a];
        err = [err, abs(a-measurements(i).angle_truth(1))];
    end
end

function [phi, err]  = estimate_phi(joint_param, measurements)
    n = size(measurements,2);
    phi = [];
    err = [];
    for i=1:n
        Rz = joint_param.R_mirror2'*measurements(i).R*joint_param.R_cam';
        a = atan2(Rz(2,1), Rz(1,1));
        phi = [phi, a];
        err = [err, abs(a-measurements(i).angle_truth(2))];
    end
end

function ejp = estimate_translation(h, ejp, measurements_fix_phi, measurements_fix_theta, theta, phi, jp)
    n = size(measurements_fix_phi,2);
    m = size(measurements_fix_theta,2);

    A = [];
    b = [];

    for i=1:n
        R_theta = [cos(theta(i)) -sin(theta(i)) 0; sin(theta(i)) cos(theta(i)) 0; 0 0 1];
        C = ejp.R_door*R_theta;
        D = C*ejp.R_mirror*ejp.R_phi;
        
        A = [A; D(1,1), D(1,2), D(1,3), C(1,1), 1, 0, 0; ...
                D(2,1), D(2,2), D(2,3), C(2,1), 0, 1, 0; ...
                D(3,1), D(3,2), D(3,3), C(3,1), 0, 0, 0];
        b = [b; measurements_fix_phi(i).t - [0;0;h]];
    end
    
    for i=1:m
        R_phi = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
        C = ejp.R_door;
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

function sim_config = get_simulation_config(noise_level, npts, npose);
    sim_config = {};
    sim_config.npts = npts;
    sim_config.npose = npose;
    sim_config.aspect_ratio = 1.6;
    sim_config.noise = noise_level/1280;
    sim_config.clouds = generate_point_cloud(sim_config.npts,20,30)';

    disp(sprintf('[Simulation Config] Noise Level: %d, #Point: %d #Pose: %d', noise_level, npts, npose));
end

function euler = axis_diff_in_euler(R1, R2)
    R = R1*R2';
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    euler = [x,y,z];
end

function print_err_report(jp,ejp, theta_err, phi_err)
    disp(sprintf('phi0 error (deg): %f', abs(ejp.phi0 - jp.phi0)/pi*180));
    disp(sprintf('Average theta error (deg): %f', mean(theta_err/pi*180)));
    disp(sprintf('Average phi error (deg): %f', mean(phi_err/pi*180)));


    R_door_diff = axis_diff_in_euler(ejp.R_door, jp.R_door);
    R_mirror_diff = axis_diff_in_euler(ejp.R_mirror, jp.R_mirror);
    R_cam_diff    = axis_diff_in_euler(ejp.R_cam, jp.R_cam);

    disp(sprintf('R_door error in ZYX (deg)\t x: %f y: %f z: %f', R_door_diff(1), R_door_diff(2), R_door_diff(3)));
    disp(sprintf('R_mirror error in ZYX (deg)\t x: %f y: %f z: %f', R_mirror_diff(1), R_mirror_diff(2), R_mirror_diff(3)));
    disp(sprintf('R_cam error in ZYX (deg)\t x: %f y: %f z: %f', R_cam_diff(1), R_cam_diff(2), R_cam_diff(3)));
    
    disp(sprintf('t_o2d error\t x: %f y: %f z: %f', ejp.t_o2d(1) - jp.t_o2d(1), ejp.t_o2d(2) - jp.t_o2d(2), ejp.t_o2d(3) - jp.t_o2d(3)));
    disp(sprintf('t_d2m error\t x: %f', ejp.t_d2m(1) - jp.t_d2m(1)));
    disp(sprintf('t_m2c error\t x: %f y: %f z: %f', ejp.t_m2c(1) - jp.t_m2c(1), ejp.t_m2c(2) - jp.t_m2c(2), ejp.t_m2c(3) - jp.t_m2c(3)));
end

