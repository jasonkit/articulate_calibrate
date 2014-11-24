function derive_recalib
syms r11 r12 r13 r21 r22 r23 r31 r32 r33 m11 m12 m13 m21 m22 m23 m31 m32 m33 t p 'real'
%R = [r11 r12 r13; r21 r22 r23; r31 r32 r33];
R_m = [m11 m12 m13; m21 m22 m23; m31 m32 m33];
R_theta = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
R_phi = [cos(p) -sin(p) 0; sin(p) cos(p) 0; 0 0 1];
R = simplify(R_theta*R_m*R_phi);

R(1,1) - r11
R(1,2) - r12
R(1,3) - r13
R(2,1) - r21
R(2,2) - r22
R(2,3) - r23
R(3,1) - r31
R(3,2) - r32
R(3,3) - r33

end
