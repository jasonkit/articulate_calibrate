function recalib_angles_plot()
load('recalib_result');

angles = get_recalibration_angles(1);
close all;
for i=1:5
    figure(i);
    j = 5*i+1;
    n = size(recalib_result{j}.theta,2);

    theta_err = abs(recalib_result{j}.theta - repmat(angles(:,1)',size(recalib_result{j}.theta,1),1));
    phi_err = abs(recalib_result{j}.phi - repmat(angles(:,2)',size(recalib_result{j}.phi,1),1));

    subplot(2,1,1);
    fill( [1:n fliplr(1:n)],  [mean(theta_err)+sqrt(var(theta_err)) fliplr(mean(theta_err)-sqrt(var(theta_err)))], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
    alpha(.1);
    hold on;
    [ax,h1,h2] = plotyy(1:n, mean(theta_err), mean(recalib_result{j}.theta)/pi*180, 'b');
    xlabel('Frame');
    ylabel(ax(1),'Error in \theta (deg)');
    ylabel(ax(2),'\theta (deg)');
    h2.Color = [0,.7,0,0.3];
    axis auto;
    set(ax,{'ycolor'},{'k';'k'})
    
    subplot(2,1,2);
    fill( [1:n fliplr(1:n)],  [mean(phi_err)+sqrt(var(phi_err)) fliplr(mean(phi_err)-sqrt(var(phi_err)))], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
    alpha(.1);
    hold on;
    [ax,h1,h2] = plotyy(1:n, mean(phi_err), mean(recalib_result{j}.phi)/pi*180, 'b');
    xlabel('Frame');
    ylabel(ax(1),'Error in \phi (deg)');
    ylabel(ax(2),'\phi (deg)');
    h2.Color = [0,.7,0,0.3];
    axis auto;
    set(ax,{'ycolor'},{'k';'k'})
end
end
