function pq = qmul(p,q)
pq = [p(1)*q(1) - dot(p(2:4),q(2:4)) p(1)*q(2:4) + q(1)*p(2:4) + cross(p(2:4), q(2:4))];

end
