function [dpdq] = dp_dq(q,sym_atlasHTM)
%DP_DQ Summary of this function goes here
%   Detailed explanation goes here
HTM = sym_atlasHTM;
H_FK = [HTM(1,4) HTM(2,4) HTM(3,4)]; %m but still sym

% set up metres w.r.t. radians Jacobian
dpdq = jacobian(H_FK, q); % still symbolic, no substitutions yet
%dpdq = jacobian(H_FK, q(3:9)); % still symbolic, no substitutions yet
end

