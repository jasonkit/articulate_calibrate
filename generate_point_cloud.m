function cloud = generate_point_cloud(n_pts, diameter, range)
coord = rand(n_pts,3);
coord(:,1) = coord(:,1)*2*pi;
coord(:,2) = coord(:,2)*pi -pi/2;
coord(:,3) = coord(:,3)*range + diameter;
cloud = zeros(n_pts,3);
[cloud(:,1), cloud(:,2), cloud(:,3)] = sph2cart(coord(:,1), coord(:,2), coord(:,3));

end
