function derive_R_cam

syms m1 m2 m3 c1 c2 c3 a1 a2 a3 b1 b2 b3 'real'
C = [c1 c2 c3; -cross([c1 c2 c3], [a1 a2 a3]); a1 a2 a3];
A = [m1 m2 m3]*C;
collect(A'-[b1;b2;b3], [c1, c2, c3])
end
