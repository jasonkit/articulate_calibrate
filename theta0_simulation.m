function theta0_simulation()
theta0 = 10*pi/180;
n = 100;
err_set = [];
for psy = [1:.1:15];
    psy
    theta0_err = [];
    parfor i = 1:n
        ejp = simulation(1, false, theta0, psy);
        theta0_estimated = ejp.theta0;
        theta0_err = [theta0_err; abs(theta0_estimated-theta0)];
    end
    theta0_err = theta0_err/pi*180;
    mean(theta0_err)
    err_set  = [err_set theta0_err];
end
    save('theta0','err_set');
end
