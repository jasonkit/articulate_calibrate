function measurements = simulate_relative_movement(joint_param, sim_config, angles, is_angle)

    if nargin == 3
        is_angle = true;
    end

    if is_angle == true
        n = size(angles,1);
    else
        Ps = angles;
        n = size(Ps,3);
    end

    measurements = [];
    prev_features = [];

    for i=1:n
        measurement = {};

        if is_angle == true
            theta = angles(i,1);
            phi = angles(i,2);

            [R_truth, t_truth] = compute_cam_pose(joint_param, theta, phi);
            P = [R_truth t_truth;0 0 0 1];
        else
            P = Ps(:,:,i);
        end

        P = inv(P);
        P = P(1:3,:);
        features = do_projection(P, sim_config.clouds, sim_config.aspect_ratio);
        num_features = size(features,1);

        while true
            features(:,1:2) = features(:,1:2) + normrnd(zeros(num_features, 2), sim_config.noise*ones(num_features,2));
            if (i > 1)
                matches = match_features(prev_features, features);
                if (size(matches,1) >= 8)
                    break;
                end
                disp('.');
            else
                break;
            end
        end
        
        if i == 1
            prev_features = features;
            measurement.R_rel = eye(3);
            
            if is_angle == true
                measurement.R_truth = R_truth; 
                measurement.t_truth = t_truth; 
                measurement.theta = theta;
                measurement.phi = phi;
            end

            measurements = [measurements; measurement];
            continue;
        end
        
        measurement.R_rel = estimate_relative_camera_rotation(matches);

        if is_angle == true
            measurement.R_truth = R_truth; 
            measurement.t_truth = t_truth; 
            measurement.theta = theta;
            measurement.phi = phi;
        end
        measurements = [measurements; measurement];
        prev_features = features;
    end
end

function matches = match_features(f1, f2)
    idx1 = 1;
    idx2 = 1;
    n1 = size(f1,1);
    n2 = size(f2,1);

    matches = [];
    while (idx1 <= n1 && idx2 <= n2)
        if f1(idx1,3) == f2(idx2,3)
            matches = [matches; f1(idx1,1:2), f2(idx2,1:2)];
            idx1 = idx1 + 1;
            idx2 = idx2 + 1;
        elseif f1(idx1,3) < f2(idx2,3)
            idx1 = idx1 + 1;
        else
            idx2 = idx2 + 1;
        end
    end
end

function R = estimate_relative_camera_rotation(matches)
    num_matches = size(matches, 1);
    [f1, T1] = normalization(matches(:,1:2)');
    [f2, T2] = normalization(matches(:,3:4)');
    inlier_ratio = 0;
    desire_p = 0.99;
    threshold = 1;
    best_inliers = [];
    best_n_inliers = 0;
    req_iter = log(1-desire_p)/log(1-(0.8)^8);
    for i = 1:1000
        train_idx = randperm(num_matches, 8);
        test_idx = setdiff(1:num_matches, train_idx);

        E = reshape(eightp(f1(:,train_idx), f2(:,train_idx)),3,3);

        n_inlier = 8;
        inliers = train_idx;

        for j = test_idx
            if (f1(:,j)'*E*f2(:,j) < threshold)
                n_inlier = n_inlier + 1;
                inliers = [inliers, j];
            end
        end

        if (n_inlier > best_n_inliers)
            best_n_inliers = n_inlier;
            best_inliers = inliers;
            req_iter = log(1-desire_p)/log(1-(best_n_inliers/num_matches)^8);
        end

        if (i > req_iter)
            break;
        end
    end

    if (req_iter > 1000)
        best_n_inliers
    end

    E = reshape(eightp(f1(:, best_inliers), f2(:, best_inliers)),3,3);
    E = T1'*E*T2;
    P = decomposeE(E, matches(best_inliers,:));
    R = P(1:3,1:3);
end

function [xyn, T] = normalization(xy)

[d n] = size(xy);

xy_centroid = sum(xy')'/n;
xy_offset = xy - (xy_centroid*ones(1,n));

xy_avg_distance = sum(sqrt(sum(xy_offset .* xy_offset)))/n;
xy_s = sqrt(2)/xy_avg_distance;

s = xy_s;
c = xy_s*xy_centroid;
T = [s 0 -c(1); 0 s -c(2); 0 0 1];

xyn = T*[xy; ones(1,n)];
end
