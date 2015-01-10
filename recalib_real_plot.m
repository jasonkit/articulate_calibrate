function recalib_real_plot()

ds1 = {'A1', 'A2'};
ds2 = {'L', 'R'};
ds3 = {'far', 'near'};
ds4 = {'left', 'right'};

ylabels = {'yaw (deg)', 'pitch (deg)', 'roll (deg)', 'x (m)', 'y (m)', 'z (m)'};

load('recalib_match');
close all;
for i = 1:2
    for j = 1:2
        figure((i-1)*2+j);
        apriltag_ds = sprintf('%s_%s', ds3{i}, ds4{j});
        load(apriltag_ds);
        poses1 = poses(:,:, match_set{(i-1)*2+j}(:,1));
        ts = ts(match_set{(i-1)*2+j}(:,1));
       
        data = zeros(size(poses1,3), 6, 5);

        for n = 1:size(data,1)
            if (j == 1)
                data(n,1:3,1) = YPR(poses1(1:3,1:3,n));
            else
                data(n,1:3,1) = YPR2(poses1(1:3,1:3,n));
            end
            data(n,4:6,1) = poses1(1:3,4,n)';
        end

        for k = 1:4
            estimate_ds = sprintf('%s_%d_%s', ds1{i}, k, ds2{j});
            load(estimate_ds);
            poses2 = poses(:,:, match_set{(i-1)*2+j}(:,2));
            
            for n = 1:size(data,1)
                if (j == 1)
                    data(n,1:3,k+1) = YPR(poses2(1:3,1:3,n));
                else
                    data(n,1:3,k+1) = YPR2(poses2(1:3,1:3,n));
                end
                data(n,4:6,k+1) = poses2(1:3,4,n)';
            end
        end

        for(k=1:6)
            subplot(6,1,k);
            hold on;
            for(m=1:5)
                plot(data(:,k,m));
            end

             
            ylabel(ylabels{k});
            xlabel('Frame');
            if (k==6)
                legend('AprilTag','Data Set 1', 'Data Set 2', 'Data Set 3', 'Data Set 4', 'Location','south', 'Orientation', 'horizontal');
                legend('boxoff');
            end
        end

        disp(apriltag_ds);
        
        data(:,4:6,:) = data(:,4:6,:)*1000;
        for k=1:4

            mean(abs(data(:,:,k+1) - data(:,:,1)))
            sqrt(var(abs(data(:,:,k+1) - data(:,:,1))))
        end
    end
end

end

function a = YPR(R)
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    a = [x,y,z];
end

function a = YPR2(R)
    x = atan2(R(3,2),R(3,3))*180/pi;
    y = atan2(-R(3,1),sqrt(R(3,2)^2 + R(3,3)^2))*180/pi;
    z = atan2(R(2,1),R(1,1))*180/pi;
    a = [x,y,z];
    
    if (a(3) < 0)
        a(3) = a(3) + 360;
    end
end

