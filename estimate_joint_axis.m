function estimate_joint_axis(b)
    for i=0:3
        for j=0:3
            denom_id = i*16+j*4 +1;
            x_id = 1 + denom_id;
            y_id = 2 + denom_id;
            z_id = 3 + denom_id;
            w = 1/sqrt(1 + (b(x_id)^2 + b(y_id)^2 + b(z_id)^2)/(b(denom_id)^2))
        end
    end

end
