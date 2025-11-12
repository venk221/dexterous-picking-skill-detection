function[j_dot, r_dot] = get_velocity(Q, lam, err)
    % Cartesian velocity
    r_dot = -lam*transpose(err);

    % Joint Velocity    
    j_dot = transpose(pinv(Q)*r_dot);
end