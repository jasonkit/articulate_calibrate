function plot_theta0_simulation()
load('theta0');
close all;
figure(1);
psy_range = [1:.1:15];
fill( [psy_range fliplr(psy_range)],  [mean(err_set)+sqrt(var(err_set)) fliplr(mean(err_set)-sqrt(var(err_set)))], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
alpha(.1);
hold on;
plot(psy_range, mean(err_set), 'b');
xlim([1,15]);
xlabel('\psi (deg)');
ylabel('Error in \theta (deg)');
end
