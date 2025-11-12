function [R] = R_HTM(q, a)
    R = [cos(q) -sin(q)*cos(a) sin(q)*sin(a);
         sin(q) cos(q)*cos(a) -cos(q)*sin(a);
         0      sin(a)         cos(a)      ];
end