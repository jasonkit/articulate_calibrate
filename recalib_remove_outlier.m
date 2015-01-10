function recalib_remove_outlier()
    load('recalib_result');
    is_updated = false;
    for i=1:26
        for j = 2:1160
            idx = find(recalib_result{i}.theta(:,j) > mean(recalib_result{i}.theta(:,j)))';
                        
            if (((length(idx) >=65 || length(idx) <= 35) && abs(mean(recalib_result{i}.theta(:,j))) > 1 ) ...
                || ...
                (length(idx) >=95 || length(idx) <= 5) && abs(mean(recalib_result{i}.theta(:,j))) > 1e-2)
                
                d = sort(recalib_result{i}.theta(:,j));
                if length(idx) >= 70
                    outlier_value = d(1);
                else
                    outlier_value = d(end);
                end
                k = find(recalib_result{i}.theta(:,j)==outlier_value);
                k = k(1);
                [i,k]
                recalib_result{i}.theta = recalib_result{i}.theta([1:k-1, k+1:end],:);
                recalib_result{i}.phi = recalib_result{i}.phi([1:k-1, k+1:end],:);
                is_updated = true;
                break;
            end
        end
    end
    if is_updated == true
        save('recalib_result', 'recalib_result');
    else
        for i=1:26
            clf;
            i
            subplot(2,1,1);
            mesh(recalib_result{i}.theta);
            subplot(2,1,2);
            mesh(recalib_result{i}.phi);
            pause;
        end
    end
end
