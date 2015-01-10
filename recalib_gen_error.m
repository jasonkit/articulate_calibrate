function recalib_gen_error()
    load('jpe_result');
    ejp = sim_result.ejps(6,1);
    m = size(sim_result.noise_range,2);
    
    load('recalib_rels');
    % noise_level = 1
    Ps = rel_measurement.Ps;
   
    load('recalib_result');

    err_data = [];
    for i = 1:m
        err_set = [];
        
        p = size(recalib_result{i}.theta,1);
        theta = recalib_result{i}.theta;
        phi = recalib_result{i}.phi;
        for j = 1:p
            disp(sprintf('i:%d j:%d',i,j));
            parfor k = 1:size(theta,2);
                T = construct_transfomation_from_angles(ejp, theta(j,k), phi(j,k));
                err = [abs(axis_diff_in_euler(T(1:3,1:3), Ps(1:3,1:3,k))), abs(T(1:3,4)-Ps(1:3,4,k))']
                err_set = [err_set; err];
            end
        end
        err_data = [err_data; mean(err_set) sqrt(var(err_set))];
    end
    save('recalib_err', 'err_data');
end
