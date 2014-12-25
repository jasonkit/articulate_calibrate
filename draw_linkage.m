function draw_linkage(joint_param, theta, phi)

clf;

R1 = joint_param.R_door*vrrotvec2mat([0 0 1 theta]);
R2 = joint_param.R_mirror*vrrotvec2mat([0 0 1 phi]);
R3 = R1*R2*joint_param.R_cam;

A1 = R1(1:3,3);
A2 = R2(1:3,3);
t0 = joint_param.t_o2d;
t1 = R1*joint_param.t_d2m + t0;
t2 = R1*R2*joint_param.t_m2c + t1;


% draw linkage
line([0,t0(1)], [0, t0(2)], [0, t0(3)], 'LineWidth', 2, 'Color','c');
line([t0(1),t1(1)], [t0(2),t1(2)], [t0(3),t1(3)], 'LineWidth', 2, 'Color','c');
line([t1(1),t2(1)], [t1(2),t2(2)], [t1(3),t2(3)], 'LineWidth', 2, 'Color', 'c');

% draw axis
line([A1(1)*3+t0(1), A1(1)*-3+t0(1)], [A1(2)*3+t0(2), A1(2)*-3+t0(2)], [A1(3)*3+t0(3), A1(3)*-3+t0(3)], 'Color','r','LineStyle','--');
line([A2(1)*3+t1(1), A2(1)*-3+t1(1)], [A2(2)*3+t1(2), A2(2)*-3+t1(2)], [A2(3)*3+t1(3), A2(3)*-3+t1(3)], 'Color','m','LineStyle','--');

% draw camera
C1 = 0.1*R3(1:3,1);
C2 = 0.1*R3(1:3,2);
C3 = 0.1*R3(1:3,3);
line([t2(1), C1(1)+t2(1)], [t2(2), C1(2)+t2(2)], [t2(3), C1(3)+t2(3)], 'Color','r','LineStyle','-');
line([t2(1), C2(1)+t2(1)], [t2(2), C2(2)+t2(2)], [t2(3), C2(3)+t2(3)], 'Color','g','LineStyle','-');
line([t2(1), C3(1)+t2(1)], [t2(2), C3(2)+t2(2)], [t2(3), C3(3)+t2(3)], 'Color','b','LineStyle','-');

axis equal;

end
