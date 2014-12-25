function plot_error_against_noise_lv()
    jp = get_joint_param();
   
    noise_range = 0:.1:5;
    num_simulation = 10;

    err_set_mean = [];
    err_set_sd = [];
    for noise_lv = noise_range
        disp(sprintf('noise lv: %f', noise_lv));
        err_all = [];
        for n = 1:num_simulation
            ejp = simulation(noise_lv, false);
            ejp.R_ref = ejp.R_door * ejp.R_mirror * ejp.R_cam;
            err = [];
            err = [err, abs(ejp.theta0 - jp.theta0)/pi*180];
            err = [err, max(abs(axis_diff_in_euler(ejp.R_door, jp.R_door)))];
            err = [err, max(abs(axis_diff_in_euler(ejp.R_mirror, jp.R_mirror)))];
            err = [err, max(abs(axis_diff_in_euler(ejp.R_cam, jp.R_cam)))];
            err = [err, max(abs(axis_diff_in_euler(ejp.R_ref, jp.R_ref)))];
            err = [err, max(abs((ejp.t_o2d-jp.t_o2d)./jp.t_o2d))*100];
            err = [err, max(abs((ejp.t_d2m-jp.t_d2m)./jp.t_d2m))*100];
            err = [err, max(abs((ejp.t_m2c-jp.t_m2c)./jp.t_m2c))*100];
            err = [err, max(abs((ejp.t_ref-jp.t_ref)./jp.t_ref))*100];

            err_all = [err_all; err];
        end
        
        err_set_mean = [err_set_mean; mean(err_all)];
        err_set_sd = [err_set_sd; sqrt(var(err_all))];
    end
    clf;
    figure(1);
    subplot(2,1,1);
    hold on;
    plotcolor = ['m', 'r', 'g', 'm', 'c'];
    for i=1:length(plotcolor)
        plot(noise_range, err_set_mean(:,i), plotcolor(i));
    end
    for i=1:length(plotcolor)
        fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,i)'+err_set_sd(:,i)' fliplr(err_set_mean(:,i)'-err_set_sd(:,i)')], plotcolor(i), 'EdgeColor', plotcolor(i), 'EdgeAlpha', .25, 'LineStyle', '--');
        alpha(.1);
    end
    legend('theta_{0}', 'R_{door}', 'R_{mirror}', 'R_{cam}', 'R_{ref}');
    ylabel('deg');
    xlabel('noise level - x/1280 as variance');

    subplot(2,1,2);
    plot(noise_range, err_set_mean(:,6), 'r');
    hold on;
    plot(noise_range, err_set_mean(:,7), 'g');
    plot(noise_range, err_set_mean(:,8), 'b');
    plot(noise_range, err_set_mean(:,9), 'm');
    legend('t_{o2d}', 't_{d2m}', 't_{m2c}', 't_{ref}');
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
