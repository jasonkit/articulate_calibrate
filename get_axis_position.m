% return rotation axis position w.r.t camera frame
function t=get_axis_position(M)

M = reshape(M(1,:),3,4);
Rf = M(:,1:3)+eye(3);
tf = M(:,4);

a = vrrotmat2vec(Rf);
a = a(1:3)/norm(a(1:3));
a = a';

basis = get_orthogonal_vector(a);

A = (eye(3)-Rf)*basis;
t = basis*(inv(A'*A)*A'*tf);

    % choose t to make one of the component = 0
    if a(2) ~= 0
        t = t - t(2)*(a/a(2));
    elseif a(3) ~= 0
        t = t - t(3)*(a/a(3));
    else
        t = t - t(1)*(a/a(1));
    end
end
