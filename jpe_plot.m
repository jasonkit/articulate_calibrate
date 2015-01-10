function jpe_plot()
    load('jpe_result');
    jp = sim_result.jp;
    ejps = sim_result.ejps;
    noise_range = sim_result.noise_range;
    n = size(noise_range,2);
    m = sim_result.num_simulation;

    err_set_mean = [];
    err_set_sd = [];
    
    for i=1:n
        err_all = [];
        for j=1:m
            ejp = ejps(i,j);
            err = [];
            err = [err, abs(axis_diff_in_euler(ejp.R_door, jp.R_door))];
            err = [err, abs(axis_diff_in_euler(ejp.R_mirror, jp.R_mirror))];
            err = [err, abs(axis_diff_in_euler(ejp.R_cam, jp.R_cam))];
            err = [err, abs(axis_diff_in_euler(ejp.R_ref, jp.R_ref))];
            err = [err, abs(((ejp.t_o2d-jp.t_o2d)./jp.t_o2d)'*100)];
            err = [err, abs(((ejp.t_d2m-jp.t_d2m)./jp.t_d2m)'*100)];
            err = [err, abs(((ejp.t_m2c-jp.t_m2c)./jp.t_m2c)'*100)];
            err = [err, abs((ejp.t_ref-jp.t_ref)')*100];
            err_all = [err_all; err];
        end
        err_set_mean = [err_set_mean; mean(err_all)];
        err_set_sd = [err_set_sd; sqrt(var(err_all))];
    end

    close all;

    ylabels = {'yaw', 'pitch', 'roll'};
    for i=1:3
        figure(i)
        for j=1:3
            k = (i-1)*3+j;
            subplot(3,1,j);
            fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,k)'+err_set_sd(:,k)' fliplr(err_set_mean(:,k)'-err_set_sd(:,k)')], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
            alpha(.1);
            hold on;
            plot(noise_range, err_set_mean(:,k), 'b');
            xlabel('noise level (pixel)');
            ylabel(sprintf('Error in %s (deg)', ylabels{j}));
        end
    end

    ylabels = {'x', 'y', 'z'};
    for i=1:3
        cnt = 1;
        for j=1:3
            if (i==1 && j==3) || (i==2 && j>1)
                continue;
            end
            k = (i-1)*3+j+12;
            figure(i+3);
            if i == 1
                subplot(2,1,cnt);
            elseif i == 3
                subplot(3,1,cnt);
            end
            cnt = cnt+1;
            fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,k)'+err_set_sd(:,k)' fliplr(err_set_mean(:,k)'-err_set_sd(:,k)')], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
            alpha(.1);
            hold on;
            plot(noise_range, err_set_mean(:,k), 'b');
            xlabel('noise level (pixel)');
            ylabel(sprintf('%% change in %s', ylabels{j}));
        end
    end

    figure(7);
    ylabels = {'yaw', 'pitch', 'roll'};
    for i=1:3
        subplot(3,1,i);
        k = 9+i;
        fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,k)'+err_set_sd(:,k)' fliplr(err_set_mean(:,k)'-err_set_sd(:,k)')], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
        alpha(.1);
        hold on;
        plot(noise_range, err_set_mean(:,k), 'b');
        xlabel('noise level (pixel)');
        ylabel(sprintf('Error in %s (deg)', ylabels{i}));
    end
    
    figure(8);
    ylabels = {'x', 'y', 'z'};
    for i=1:3
        subplot(3,1,i);
        k = 21+i;
        fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,k)'+err_set_sd(:,k)' fliplr(err_set_mean(:,k)'-err_set_sd(:,k)')], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
        alpha(.1);
        hold on;
        plot(noise_range, err_set_mean(:,k), 'b');
        xlabel('noise level (pixel)');
        ylabel(sprintf('Error in %s (cm)', ylabels{i}));
    end

    err_set_mean(6,:) + err_set_sd(6,:)
    err_set_mean(26,:) + err_set_sd(26,:)

end

function euler = axis_diff_in_euler(R1, R2)
    R = R1*R2';
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    euler = [x,y,z];
end
