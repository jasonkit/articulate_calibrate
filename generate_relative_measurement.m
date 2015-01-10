function generate_relative_measurement()

    load('jpe_result');
    
    m = size(sim_result.noise_range,2);
    num_simulation = 100;

    rel_measurement = {};

    angles = get_recalibration_angles(1);
    jp = sim_result.jp;
    Ps = posesFromAngles(jp,angles);
    
    rel_measurement.angles = angles;
    rel_measurement.Ps = Ps;
    rel_measurement.Rrels = [];
    for i = 1:m
        disp(sim_result.noise_range(i));
        sim_config = get_simulation_config(sim_result.noise_range(i), 1000, 50, false);
        Rrels = [];
        parfor j = 1:num_simulation
            Rrel = {};
            Rrel.measurements = simulate_relative_movement(jp, sim_config, Ps, false);
            Rrels = [Rrels, Rrel];
        end
        rel_measurement.Rrels = [rel_measurement.Rrels; Rrels];
    end

    save('recalib_rels','rel_measurement');
end

function Ps = posesFromAngles(jp, angles)
    Ps = zeros(4,4,size(angles,1));

    for i=1:size(angles,1)
        theta = angles(i,1);
        phi = angles(i,2);

        [R, t] = compute_cam_pose(jp, theta, phi);
        Ps(:,:,i) = [R t;0 0 0 1];
    end
end

