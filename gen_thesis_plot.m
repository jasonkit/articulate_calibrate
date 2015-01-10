% joint parameters estimation
jpe_simulation;
jpe_plot;

% theta0 against psi
theta0_simulation;
theta0_plot;

% recalibration
generate_relative_measurement;
recalib_simulation;
recalib_remove_outlier;
recalib_err_plot;
recalib_angles_plot;

% real recalibration
recalib_match;
recalib_real_plot;
