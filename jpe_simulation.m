function sim_result = run_jpe()
    theta0 = 0; %pi*10/180;
    jp = get_joint_param(theta0);
   
    noise_range = 0:.2:5;
    num_simulation = 200;

    n = size(noise_range,2);
    
    sim_result = {};
    sim_result.jp = jp;
    sim_result.noise_range = noise_range;
    sim_result.num_simulation = num_simulation;
    sim_result.ejps = [];
    
    for i = 1:n
        noise_lv = noise_range(i);
        disp(sprintf('noise lv: %f', noise_lv));
        ejps = [];
        parfor j = 1:num_simulation
            ejp = simulation(noise_lv, false, theta0, 0);
            ejp.R_ref = ejp.R_door * ejp.R_mirror * ejp.R_cam;
            ejps = [ejps, ejp]; 
        end
        sim_result.ejps = [sim_result.ejps; ejps];
    end

    save('jpe_result','sim_result');
end
