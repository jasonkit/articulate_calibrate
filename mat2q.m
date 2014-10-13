function q = mat2q(M)

    q = zeros(1,4);
    tr = trace(M);

    if (tr > 0) 
      S = sqrt(tr+1.0) * 2; % S=4*qw 
      q(1) = 0.25 * S;
      q(2) = (M(3,2) - M(2,3)) / S;
      q(3) = (M(1,3) - M(3,1)) / S; 
      q(4) = (M(2,1) - M(1,2)) / S; 
    elseif ((M(1,1) > M(2,2)) && (M(1,1) > M(3,3)))
      S = sqrt(1.0 + M(1,1) - M(2,2) - M(3,3)) * 2; % S=4*qx 
      q(1) = (M(3,2) - M(2,3)) / S;
      q(2) = 0.25 * S;
      q(3) = (M(1,2) + M(2,1)) / S; 
      q(4) = (M(1,3) + M(3,1)) / S; 
    elseif (M(2,2) > M(3,3))
      S = sqrt(1.0 + M(2,2) - M(1,1) - M(3,3)) * 2; % S=4*qy
      q(1) = (M(1,3) - M(3,1)) / S;
      q(2) = (M(1,2) + M(2,1)) / S; 
      q(3) = 0.25 * S;
      q(4) = (M(2,3) + M(3,2)) / S; 
    else
      S = sqrt(1.0 + M(3,3) - M(1,1) - M(2,2)) * 2; % S=4*qz
      q(1) = (M(2,1) - M(1,2)) / S;
      q(2) = (M(1,3) + M(3,1)) / S;
      q(3) = (M(2,3) + M(3,2)) / S;
      q(4) = 0.25 * S;
    end

    if (q(1) < 0)
        q = -q;
    end

end
