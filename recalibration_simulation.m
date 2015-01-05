function recalibration_simulation()
    load('jpe_result');
    [m, n] = size(sim_result.ejps);
    
    sim_config = get_simulation_config(sim_result.noise_range(1), 1000, 50, false);
    simulation_recalibration(sim_config, sim_result.jp, sim_result.ejps(1,1));
end

function simulation_recalibration(sim_config, jp, ejp)
    T = 560;
    t = [0:T-1]';
    %angles = [(cos(2*pi*t*(4/T))-1)*35, (cos(2*pi*t*(2/T))-1)*-22.5]*pi/180;
    %angles = [(cos(2*pi*t*(2/T))-1)*35, zeros(T,1)]*pi/180;

    angles = [-(0:2:69)', zeros(35,1); ...
              (0:2:69)'-70, zeros(35,1); ...
              -(0:2:69)', zeros(35,1); ...
              (0:2:69)'-70, zeros(35,1); ...
              zeros(25,1), (0:2:49)'; ...
              zeros(25,1), -(0:2:49)'+50;...
              zeros(25,1), (0:2:49)'; ...
              zeros(25,1), -(0:2:49)'+50;...
              zeros(25,1), (0:2:49)'; ...
              zeros(25,1), -(0:2:49)'+50]*pi/180;
    angles = [angles; angles;];
    angles = [angles; angles;];
    %angles = [angles; angles;];
    %angles = [angles; angles;];
   
    T = size(angles,1)

    measurements = simulate_relative_movement(jp, sim_config, angles);
   
    R = jp.R_ref;

    last_theta = 0;
    last_phi = 0;
    last_movement = 0;
    data = [];
    for i = 1:T
        R_cur = R * measurements(i).R_rel;
        [theta, phi, last_movement] = estimate_joint_angle(ejp, measurements(i).R_rel, R_cur, last_theta, last_phi, last_movement, true, false);
        H = construct_transfomation_from_angles(ejp, theta, phi);
        R = H(1:3,1:3);
        data = [data; [theta, measurements(i).theta, phi, measurements(i).phi]*180/pi, axis_diff_in_euler(R,measurements(i).R_truth)];
        last_theta = theta;
        last_phi = phi;
    end

    clf;
    subplot(5,2,2);
    plot(data(:,1),'r');
    hold on;
    plot(data(:,2),'g');
    legend('\theta_{estimated}', '\theta_{truth}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,4);
    plot(abs(data(:,1)-data(:,2)));
    legend('\theta_{error}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,6);
    plot(data(:,3),'r');
    hold on;
    plot(data(:,4),'g');
    legend('\phi_{estimated}', '\phi_{truth}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,8);
    plot(abs(data(:,3)-data(:,4)));
    legend('\phi_{error}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,10);
    plot(data(:,5),'r');
    hold on;
    plot(data(:,6),'g');
    plot(data(:,7),'b');
    legend('{Err}_{yaw}', '{Err}_{pitch}', '{Err}_{roll}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

%%00------


    R = jp.R_ref;

    last_theta = 0;
    last_phi = 0;
    last_movement = 0;
    data = [];
    for i = 1:T
        R_cur = R * measurements(i).R_rel;
        [theta, phi, last_movement] = estimate_joint_angle(ejp, measurements(i).R_rel, R_cur, last_theta, last_phi, last_movement, true, true);
        H = construct_transfomation_from_angles(ejp, theta, phi);
        R = H(1:3,1:3);
        data = [data; [theta, measurements(i).theta, phi, measurements(i).phi]*180/pi, axis_diff_in_euler(R,measurements(i).R_truth)];
        last_theta = theta;
        last_phi = phi;
    end
    
    subplot(5,2,1);
    plot(data(:,1),'r');
    hold on;
    plot(data(:,2),'g');
    legend('\theta_{estimated}', '\theta_{truth}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,3);
    plot(abs(data(:,1)-data(:,2)));
    legend('\theta_{error}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,5);
    plot(data(:,3),'r');
    hold on;
    plot(data(:,4),'g');
    legend('\phi_{estimated}', '\phi_{truth}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,7);
    plot(abs(data(:,3)-data(:,4)));
    legend('\phi_{error}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');

    subplot(5,2,9);
    plot(data(:,5),'r');
    hold on;
    plot(data(:,6),'g');
    plot(data(:,7),'b');
    legend('{Err}_{yaw}', '{Err}_{pitch}', '{Err}_{roll}');
    legend('boxoff');
    ylabel('deg');
    xlabel('Frame');


end

function [theta, phi, last_movement] = estimate_joint_angle(ejp, R_rel, R, last_theta, last_phi, last_movement, is_left, debug)
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

    if (debug == true)
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
