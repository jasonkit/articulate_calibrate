function derive_jacobian()

syms b1 b2 b3 b4 theta phi 'real';
syms rw rx ry rz dw dx dy dz mw mx my mz cw cx cy cz 'real';
syms tcx tcy tcz tmx tmy tmz tdx tdy tdz 'real';

qc = [cw cx cy cz];
qphi = [cos(phi/2) 0 0 sin(phi/2)];
qm = [mw mx my mz];
qtheta = [cos(theta/2) 0 0 sin(theta/2)];
qd = [dw dx dy dz];
simplify(qmul(qtheta,qc))
q1 = qmul(qd,qtheta);
q2 = qmul(qm,qphi);
q1c = q1;
q2c = q2;
q1c(2:4) = -q1c(2:4);
q2c(2:4) = -q2c(2:4);

tc = [0 tcx tcy tcz];
tm = [0 tmx tmy tmz];
td = [0 tdx tdy tdz];

f1 = expand(qmul(q1,qmul(q2,qmul(tc,qmul(q2c,q1c)))) + qmul(q1,qmul(tm,q1c)) + td);
f2 = expand(qmul(q1,qmul(q2,qc)));

f1(2)

if 0
J11 = collect(simplify(diff(f1(2),theta)), [sin(theta), cos(theta), sin(phi), cos(phi), sin(phi+theta),sin(phi-theta),cos(phi+theta),cos(phi-theta)])
J12 = collect(simplify(diff(f1(2),phi)), [sin(theta), cos(theta), sin(phi), cos(phi), sin(phi+theta),sin(phi-theta),cos(phi+theta),cos(phi-theta)])

J21 = collect(simplify(diff(f1(3),theta)), [sin(theta), cos(theta), sin(phi), cos(phi), sin(phi+theta),sin(phi-theta),cos(phi+theta),cos(phi-theta)])
J22 = collect(simplify(diff(f1(3),phi)), [sin(theta), cos(theta), sin(phi), cos(phi), sin(phi+theta),sin(phi-theta),cos(phi+theta),cos(phi-theta)])

J31 = collect(simplify(diff(f1(4),theta)), [sin(theta), cos(theta), sin(phi), cos(phi), sin(phi+theta),sin(phi-theta),cos(phi+theta),cos(phi-theta)])
J32 = collect(simplify(diff(f1(4),phi)), [sin(theta), cos(theta), sin(phi), cos(phi), sin(phi+theta),sin(phi-theta),cos(phi+theta),cos(phi-theta)])
end

if 0
J41 = collect(simplify(diff(f2(1),theta)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])
J42 = collect(simplify(diff(f2(1),phi)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])

J51 = collect(simplify(diff(f2(2),theta)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])
J52 = collect(simplify(diff(f2(2),phi)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])

J61 = collect(simplify(diff(f2(3),theta)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])
J62 = collect(simplify(diff(f2(3),phi)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])

J71 = collect(simplify(diff(f2(4),theta)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])
J72 = collect(simplify(diff(f2(4),phi)), [sin(phi/2 - theta/2),sin(phi/2 + theta/2),cos(phi/2 - theta/2),cos(phi/2 + theta/2)])
end
end
