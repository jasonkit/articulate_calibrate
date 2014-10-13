function measurement = get_measurement(joint_param, sim_config, theta, phi)
    measurement = {};

    [R_truth, t_truth] = compute_cam_pose(joint_param, theta, phi);
    P = [R_truth t_truth;0 0 0 1];
    P = inv(P);
    P = P(1:3,:);
    features = do_projection(P, sim_config.clouds, sim_config.aspect_ratio);
    num_features = size(features,1);
 
    if(num_features < 13)
        return;
    end

    features(:,1:2) = features(:,1:2) + normrnd(zeros(num_features, 2), sim_config.noise*ones(num_features,2));
    [R_measure, t_measure, inlier_ratio] = estimate_camera_pose(features, sim_config.clouds);
    q_truth = mat2q(R_truth);
    q_measure = mat2q(R_measure);
    
    measurement.R = R_measure;
    measurement.t = t_measure;
    %measurement.R = R_truth;
    %measurement.t = t_truth;

    measurement.err = [norm(q_truth-q_measure) norm(t_truth-t_measure)];
    measurement.angle_truth = [theta, phi];
end
