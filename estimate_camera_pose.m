function [R,t, inlier_ratio] = estimate_camera_pose (features, cloud)
    num_features = size(features, 1);
    inlier_ratio = 12/num_features;
    desire_p = 0.99;
    threshold = 1;
    best_inliers = [];
    best_n_inliers = 12;
    req_iter = log(1-desire_p)/log(1-(best_n_inliers/num_features)^12);
    for i = 1:1000
        train_idx = randperm(num_features, 12);
        test_idx = setdiff(1:num_features, train_idx);

        [R_train, t_train] = estimate_camera_pose_internal(features(train_idx,:), cloud);

        n_inlier = 12;
        inliers = train_idx;

        for j = test_idx
            if (reproj_err(R_train, t_train, cloud(:,features(j,3)), features(j,1:2)') < threshold)
                n_inlier = n_inlier + 1;
                inliers = [inliers, j];
            end
        end

        if (n_inlier > best_n_inliers)
            best_n_inliers = n_inlier;
            best_inliers = inliers;
            req_iter = log(1-desire_p)/log(1-(best_n_inliers/num_features)^12);
        end

        if (i > req_iter)
            break;
        end
    end

    if (req_iter > 1000)
        best_n_inliers
    end

    [R,t] = estimate_camera_pose_internal(features(best_inliers,:), cloud);
    inlier_ratio = best_n_inliers/num_features;
end

function [R,t] = estimate_camera_pose_internal(features, cloud)
    n = size(features,1);
   
    X = cloud(:, features(:,3));
    x = features(:,1:2)'; ones(1, n);

    [xn, Xn, Tn, Un] = normalization(x, X);

    B1 = Xn';
    B2 = (xn(1,:)'*ones(1,4)) .* B1;
    B3 = (xn(2,:)'*ones(1,4)) .* B1;
    A = [zeros(n,4) -B1 B3;B1 zeros(n,4) -B2];
    [U S V] = svd(A);
    P = reshape(V(:,end),4,3)';

    P = inv(Tn)*P*Un;

    M = P(1:3,1:3);
    [R_inv, K_inv] = qr(inv(M));
    R = inv(R_inv);
    K = inv(K_inv);

    K_scale = abs(K(3,3));

    K = K/K(3,3);
    if det(R) > 0
        if K(1,1) < 0
            K = K*diag([-1, 1, 1]);
            R = diag([1, -1, -1])*R;
        end

        if K(2,2) < 0
            K = K*diag([1, -1, 1]);
            R = diag([-1, 1, -1])*R;
        end
    else
        if K(1,1) < 0
            K = K*diag([-1, 1, 1]);
            R = diag([-1, 1, 1])*R;
        end

        if K(2,2) < 0
            K = K*diag([1, -1, 1]);
            R = diag([1, -1, 1])*R;
        end
    end
    
    R = R';
    t = ((1/K_scale)*R)*P(1:3,4);

    if (det(M) > 0)
        t = -t;
    end
end

function err = reproj_err(R,t, pt_3d, pt_2d)
P = [R t; 0 0 0 1];
reproj_features = inv(P)*[pt_3d; 1];
reproj_features = reproj_features ./ reproj_features(3);
err = norm(reproj_features(1:2) - pt_2d);
end

function [xyn, XYZn, T, U] = normalization(xy, XYZ)

[d n] = size(xy);

%data normalization
%first compute centroid
xy_centroid = sum(xy')'/n;
XYZ_centroid = sum(XYZ')'/n;

%then, compute scale
xy_offset = xy - (xy_centroid*ones(1,n));
xy_avg_distance = sum(sqrt(sum(xy_offset .* xy_offset)))/n;
xy_s = sqrt(2)/xy_avg_distance;

XYZ_offset = XYZ - (XYZ_centroid*ones(1,n));
XYZ_avg_distance = sum(sqrt(sum(XYZ_offset .* XYZ_offset)))/n;
XYZ_s = sqrt(3)/XYZ_avg_distance;

%create T and U transformation matrices

s = xy_s;
c = xy_s*xy_centroid;
T = [s 0 -c(1); 0 s -c(2); 0 0 1];

s = XYZ_s;
c = XYZ_s*XYZ_centroid;
U = [s 0 0 -c(1); 0 s 0 -c(2); 0 0 s -c(3); 0 0 0 1];

%and normalize the points according to the transformations
xyn = T*[xy; ones(1,n)];
XYZn = U*[XYZ; ones(1,n)];

end

