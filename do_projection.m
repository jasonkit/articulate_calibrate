function features = do_projection(P, cloud, aspect_ratio)
    cloud = P*[cloud; ones(1, size(cloud,2))];

    cloud_idx = find(cloud(3,:) > 0);

    features = cloud(1:3,cloud_idx)./(ones(3,1)*cloud(3,cloud_idx));

    feature_idx = find((abs(features(1,:)) < 1) .* (abs(features(2,:))<(1/aspect_ratio)));
    features = [features(1:2,feature_idx)' cloud_idx(feature_idx)'];
end
