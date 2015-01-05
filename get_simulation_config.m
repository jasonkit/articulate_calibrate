function sim_config = get_simulation_config(noise_level, npts, npose, verbose);
    sim_config = {};
    sim_config.npts = npts;
    sim_config.npose = npose;
    sim_config.aspect_ratio = 1.6;
    sim_config.noise = noise_level/1280;
    sim_config.clouds = generate_point_cloud(sim_config.npts,20,30)';
    
    if (verbose)
        disp(sprintf('[Simulation Config] Noise Level: %d, #Point: %d #Pose: %d', noise_level, npts, npose));
    end
end
