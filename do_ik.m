% dxi = invJ*dtheta
% theta0 is the initial guess for theta
function [theta, phi] = do_ik(joint_param, goal_xi, theta0, phi0)
theta = theta0;
phi = phi0;
alpha = 1e-3;

for i=1:1000 

    if (i >1)
        last_xi = cur_xi;
    else
        last_xi = nan;
    end

    cur_xi = do_fk(joint_param, theta, phi);
    
    xi_diff = goal_xi - cur_xi;
    
    if (xi_diff'*xi_diff < 1e-4)
        %i
        %xi_diff'*xi_diff
        break;
    elseif sum(isnan(last_xi)) == 0 && norm(last_xi-cur_xi)<1e-5
        %i
        %xi_diff'*xi_diff
        break; 
    end
    
    invJ = pinv(get_jacobian(joint_param, theta, phi));
    angle_diff = invJ*xi_diff;
    theta = theta + alpha*angle_diff(1);
    phi = phi + alpha*angle_diff(2);
end
 
end

function xi = do_fk(joint_param, theta, phi)
q1 = qmul(joint_param.q_door, [cos(theta/2), 0 ,0 sin(theta/2)]);
q2 = qmul(joint_param.q_mirror, [cos(phi/2), 0, 0, sin(phi/2)]);
qc = joint_param.q_cam;
q1c = q1;
q2c = q2;
q1c(2:4) = -q1c(2:4);
q2c(2:4) = -q2c(2:4);

tc = [0, joint_param.t_m2c'];
tm = [0, joint_param.t_d2m'];
td = [0, joint_param.t_o2d'];

f1 = (qmul(q1,qmul(q2,qmul(tc,qmul(q2c,q1c)))) + qmul(q1,qmul(tm,q1c)) + td)';
f2 = (qmul(q1,qmul(q2,qc)))';
xi = [f1(2:4);f2];
end
