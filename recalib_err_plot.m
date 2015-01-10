function recalib_err_plot()
    close all;
    load('recalib_err');
    noise_range = 0:.2:5;

    err_data(:,4:6) = err_data(:,4:6)*1000;
    err_data(:,10:12) = err_data(:,10:12)*1000;
    err_set_mean = err_data(:,1:6);
    err_set_sd = err_data(:,7:end);

    ylabels = {'yaw', 'pitch', 'roll'};
    figure(1);
    for k=1:3
        subplot(3,1,k);
        fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,k)'+err_set_sd(:,k)' fliplr(err_set_mean(:,k)'-err_set_sd(:,k)')], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
        alpha(.1);
        hold on;
        plot(noise_range, err_set_mean(:,k), 'b');
        xlabel('noise level (pixel)');
        ylabel(sprintf('Error in %s (deg)', ylabels{k}));
    end
    
    ylabels = {'x', 'y', 'z'};
    figure(2);
    for i=1:3
        k = i+3
        subplot(3,1,i);
        fill( [noise_range fliplr(noise_range)],  [err_set_mean(:,k)'+err_set_sd(:,k)' fliplr(err_set_mean(:,k)'-err_set_sd(:,k)')], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
        alpha(.1);
        hold on;
        plot(noise_range, err_set_mean(:,k), 'b');
        xlabel('noise level (pixel)');
        ylabel(sprintf('Error in %s (mm)', ylabels{i}));
    end
end
