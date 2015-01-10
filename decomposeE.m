% Decompose the essential matrix
% Return P = [R|t] which relates the two views
% Yu will need the point correspondences to find the correct solution for P
function P = decomposeE(E, matches)
n = size(matches,1);
x1s = [matches(:,1:2), ones(n,1)]';
x2s = [matches(:,3:4), ones(n,1)]';

[U S V] = svd(E);
W = [0 -1 0;1 0 0; 0 0 1]; 

R1 = U*W*V';

%ensure right-hand coordinate
if (det(R1) < 0)
    [U S V] = svd(-E);
    R1 = U*W*V';
end

R2 = U*W'*V';
t = U(:,3);
t = t/norm(t);

PP = zeros(3,4,4);
P1 = [eye(3) zeros(3,1)];

% all 4 possible projection
PP(:,:,1) = [R1 t];
PP(:,:,2) = [R1 -t];
PP(:,:,3) = [R2 t];
PP(:,:,4) = [R2 -t];

correctP_index = 0;
k = 1;
while correctP_index == 0
    for i=1:4  
        [XS error] = linearTriangulation(P1, [x1s(:,k)], PP(:,:,i), [x2s(:,k)]);
        m1 = P1*XS;
        m2 = PP(:,:,i)*XS;
        % correct project should be +ve z component (in front of camera)
        % for both projection.
        if (sign(m1(3)) == 1) && (sign(m2(3)) == 1)
            correctP_index = i;
            break;
        end
    end
    k = k+1;
end

P = PP(:,:,correctP_index);
end
