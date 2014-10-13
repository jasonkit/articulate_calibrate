function R = construct_orientation_matrix(z, x)
    axis_z = z;
    axis_z = axis_z/norm(axis_z);
    axis_x = x;
    axis_y = cross(axis_z,axis_x);
    axis_y = axis_y/norm(axis_y);
    axis_x = cross(axis_y,axis_z);
    axis_x = axis_x/norm(axis_x);
    R = [axis_x axis_y axis_z];
end
