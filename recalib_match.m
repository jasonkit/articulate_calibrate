function recalib_match()
    
    ts1 = {};
    load('far_left');
    ts1{1} = ts;
    load('far_right');
    ts1{2} = ts;
    load('near_left');
    ts1{3} = ts;
    load('near_right');
    ts1{4} = ts;
    
    ts2 = {};
    load('A1_1_L');
    ts2{1} = ts;
    load('A1_1_R');
    ts2{2} = ts;
    load('A2_1_L');
    ts2{3} = ts;
    load('A2_1_R');
    ts2{4} = ts;

    match_set = {};

    parfor i = 1:4
        p = 1;
        q = 1;
        matches = [];

        while(p <= size(ts1{i},2) && q <= size(ts2{i},2))
            if (ts1{i}(p) == ts2{i}(q))
                matches = [matches; p q];
                p = p+1;
                q = q+1;
            elseif (ts1{i}(p) < ts2{i}(q))
                p = p+1;
            else
                q = q+1;
            end
        end
        match_set{i} = matches;
    end

    save('recalib_match', 'match_set');
end
