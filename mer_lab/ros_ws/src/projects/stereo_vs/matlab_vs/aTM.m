% return homogeneous transform matrix between two links of DH parameters
function T_x_y = aTM(L)

% L(4) = theta, L(3) = alpha, L(2) = d, L(1) = a
    T_x_y = [cos(L(4))          -1*sin(L(4))          0            L(1);
             sin(L(4))*cos(L(3)) cos(L(4))*cos(L(3)) -1*sin(L(3)) -L(2)*sin(L(3));
             sin(L(4))*sin(L(3)) cos(L(4))*sin(L(3))  cos(L(3))    L(2)*cos(L(3));
             0                   0                    0            1];
end