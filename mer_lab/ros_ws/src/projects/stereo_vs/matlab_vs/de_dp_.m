function [ldedp,rdedp] = de_dp_(c, varphi)
%DE_DP Summary of this function goes here
%   Detailed explanation goes here
%% Obtain Jacobian from e to p
syms xW yW zW
X_W = [xW; yW; zW]; % symbolic

sym e_;

% get symbolic left and right camera errors as a function of X_W
le_ = -1*I_c_i_(X_W,1,c,varphi);
re_ = -1*I_c_i_(X_W,2,c,varphi);
% set up pixel w.r.t. metres Jacobian for left and right
ldedp = jacobian(le_,X_W);
rdedp = jacobian(re_,X_W);
end