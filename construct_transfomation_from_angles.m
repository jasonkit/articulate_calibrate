function H = construct_transfomation_from_angles(jp, theta, phi)
    R1 = jp.R_door * rotation_matrix_from_angle(theta);
    R2 = jp.R_mirror * rotation_matrix_from_angle(phi);
    R = R1*R2*jp.R_cam;
    t = R1*R2*jp.t_m2c + R1*jp.t_d2m + jp.t_o2d;
    H = [R t; 0 0 0 1];
end
