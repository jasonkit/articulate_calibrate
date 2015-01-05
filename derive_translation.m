function derive_translation()

syms c11 c12 c13 c21 c22 c23 c31 c32 c33 d11 d12 d13 d21 d22 d23 d31 d32 d33 t1 t2 t3 t4 t5 t6 h 'real';

C = [c11 c12 c13; c21 c22 c23; c31 c32 c33];
D = [d11 d12 d13; d21 d22 d23; d31 d32 d33];
m2c = [t1;t2;t3];
d2m = [t4;0;0];
o2d = [t5;t6;h];
A = collect(D*m2c + C*d2m + o2d, [t1,t2,t3,t4,t5,t6,h])
end
