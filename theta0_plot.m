function theta0_plot()
load('theta0');
close all;
figure(1);

psy_range = [1:.1:15];
subplot(2,1,1)
fill( [psy_range fliplr(psy_range)],  [mean(err_set)+sqrt(var(err_set)) fliplr(mean(err_set)-sqrt(var(err_set)))], 'b', 'EdgeColor', 'b', 'EdgeAlpha', .25, 'LineStyle', '--');
alpha(.1);
hold on;
plot(psy_range, mean(err_set), 'b');
xlim([1,15]);
xlabel('\psi (deg)');
ylabel('Error in \theta (deg)');

subplot(2,1,2)
plot(psy_range, (mean(err_set)).^-1, 'b');
xlim([1,15]);
xlabel('log \psi');
ylabel('Inverted error in \theta (deg^{-1})');
end
