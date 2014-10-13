function plot_error_against_noise_lv()
    jp = get_joint_param();
   
    noise_range = 0:.1:2;
    err_set = [];
    for noise_lv = noise_range
        ejp = simulation(noise_lv);
        err = [];
        err = [err, abs(ejp.phi0 - jp.phi0)/pi*180];
        err = [err, max(abs(axis_diff_in_euler(ejp.R_door, jp.R_door)))];
        err = [err, max(abs(axis_diff_in_euler(ejp.R_mirror, jp.R_mirror)))];
        err = [err, max(abs(axis_diff_in_euler(ejp.R_cam, jp.R_cam)))];
        err = [err, max(abs((ejp.t_o2d-jp.t_o2d)./jp.t_o2d))*100];
        err = [err, max(abs((ejp.t_d2m-jp.t_d2m)./jp.t_d2m))*100];
        err = [err, max(abs((ejp.t_m2c-jp.t_m2c)./jp.t_m2c))*100];
        err_set = [err_set; err];
    end
    clf;
    figure(1);
    subplot(2,1,1);
    plot(noise_range, err_set(:,1), 'r');
    hold on;
    plot(noise_range, err_set(:,2), 'g');
    plot(noise_range, err_set(:,3), 'b');
    plot(noise_range, err_set(:,4), 'm');
    legend('phi_{0}', 'R_{door}', 'R_{mirror}', 'R_{cam}');
    ylabel('deg');
    xlabel('noise level - x/1280 as variance');

    subplot(2,1,2);
    plot(noise_range, err_set(:,4), 'r');
    hold on;
    plot(noise_range, err_set(:,5), 'g');
    plot(noise_range, err_set(:,6), 'b');
    legend('t_{o2d}', 't_{d2m}', 't_{m2c}');
    ylabel('% change');
    xlabel('noise level - x/1280 as variance');
end

function euler = axis_diff_in_euler(R1, R2)
    R = R1*R2';
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    euler = [x,y,z];
end
