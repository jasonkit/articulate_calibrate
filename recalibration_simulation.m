function recalibration_simulation()
    load('jpe_result');
    jp = sim_result.jp;
    % noise_level = 1
    ejp = sim_result.ejps(6,1);
    
    load('recalib_rels');
    angles = rel_measurement.angles;
    Ps = rel_measurement.Ps;
    Rrels = rel_measurement.Rrels;
    [m,n] = size(Rrels);
    
    recalib_result = {};
    for i=1:m
        disp(sprintf('i=%d',i));
        theta = [];
        phi = [];
        parfor j=1:n;
            trajectory = {};
            angles = simulation_recalibration(Rrels(i,j).measurements, jp, ejp);
            theta = [theta; angles(:,1)'];
            phi = [phi; angles(:,2)'];
        end
        recalib_result{i}.theta = theta;
        recalib_result{i}.phi = phi;
    end

    save('recalib_result', 'recalib_result');
end

function angles = simulation_recalibration(measurements, jp, ejp)
   
    N = size(measurements,1);
    R = jp.R_ref;

    last_theta = 0;
    last_phi = 0;
    last_movement = 0;
    angles = [];
    for i = 1:N
        R_cur = R * measurements(i).R_rel;
        [theta, phi, last_movement] = estimate_joint_angle(ejp, measurements(i).R_rel, R_cur, last_theta, last_phi, last_movement, true, false);
        H = construct_transfomation_from_angles(ejp, theta, phi);
        R = H(1:3,1:3);
        angles = [angles; theta, phi];
        last_theta = theta;
        last_phi = phi;
    end 
end

function [theta, phi, last_movement] = estimate_joint_angle(ejp, R_rel, R, last_theta, last_phi, last_movement, is_left, simple_mode)

    R_theta_predict = ejp.R_door' * R * (ejp.R_mirror * rotation_matrix_from_angle(last_phi) * ejp.R_cam)';
    R_phi_predict = (ejp.R_door * rotation_matrix_from_angle(last_theta) * ejp.R_mirror)' * R * ejp.R_cam';
    theta_predict = atan2(R_theta_predict(2,1), R_theta_predict(1,1));
    phi_predict = atan2(R_phi_predict(2,1), R_phi_predict(1,1));

    direction = vrrotmat2vec(R_rel);
    direction = -sign(direction(3))*direction(4);

    if (is_left == false)
        direction = -direction;
    end

    if (last_movement == 0 && direction < 0) || (last_movement == 2 && direction < 0 && phi_predict < 0)
        last_movement = 1;
    elseif (last_movement == 0 && direction > 0) || (last_movement == 1 && direction > 0 && theta_predict > 0)
        last_movement = 2;
    end

    if (simple_mode == true)
        last_movement = 0;
    end

    if last_movement == 1
        R_theta = ejp.R_door' * R * (ejp.R_mirror * ejp.R_cam)';
        theta = atan2(R_theta(2,1)-R_theta(1,2), R_theta(1,1)+R_theta(2,2));
        phi = 0;
    elseif last_movement == 2
        R_phi = (ejp.R_door * ejp.R_mirror)' * R * ejp.R_cam';
        phi = atan2(R_phi(2,1)-R_phi(1,2), R_phi(1,1)+R_phi(2,2));
        theta = 0;
    else
        r = ejp.R_door' * R * ejp.R_cam';
        m = ejp.R_mirror;
        A1 = [m(1,3), -m(2,3);m(2,3), m(1,3)];
        A2 = [m(3,1), m(3,2);m(3,2), -m(3,1)];
        b1 = [r(1,3); r(2,3)];
        b2 = [r(3,1); r(3,2)];
        x1 = inv(A1)*b1;
        x2 = inv(A2)*b2;
        theta = atan2(x1(2),x1(1));
        phi = atan2(x2(2),x2(1));
    end

end
