function plot_depth(k)
depth = sort(load('depth.txt'));
range = load('depth_range.txt');
range = [range(:,1)+1, range(:,2)];
D = depth(range(k,1):range(k,2));
clf;
if(k > 1)
    histogram(D,100);
else
    histogram(D);
end
m = mean(D)
hold on;
ylim=get(gca,'ylim');
line([m m], ylim, 'Color', 'r');
hold off;
end