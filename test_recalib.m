function test_recalib(sim_config, iter, jp, ejp, theta, phi)
    err_theta_all = [];
    err_phi_all = [];
    for i = 1:iter
        err_theta = [];
        err_phi = [];
        for j = 1:length(theta)
            measurement = get_measurement(jp, sim_config, theta(j), phi(j));
            n = size(measurement, 2);

            m = ejp.R_mirror;
            r = ejp.R_door'*measurement.R*ejp.R_cam';

            A_theta = [m(1,3), -m(2,3); m(2,3), m(1,3)];
            b_theta = [r(1,3); r(2,3)];
            c_theta = inv(A_theta)*b_theta;
            c_theta = c_theta/norm(c_theta);
            theta_estimated = atan2(c_theta(2), c_theta(1));
            err_theta = [err_theta abs(theta_estimated-theta(j))];

            A_phi = [m(3,1), m(3,2); m(3,2), -m(3,1)];
            b_phi = [r(3,1); r(3,2)];
            c_phi = inv(A_phi)*b_phi;
            c_phi = c_phi/norm(c_phi);
            phi_estimated = atan2(c_phi(2), c_phi(1));
            err_phi = [err_phi abs(phi_estimated-phi(j))];
        end

        err_theta_all = [err_theta_all; err_theta];
        err_phi_all = [err_phi_all; err_phi];
    end

    mean(err_theta_all)/pi*180
    var(err_theta_all)/pi*180
    mean(err_phi_all)/pi*180
    var(err_phi_all)/pi*180

end
