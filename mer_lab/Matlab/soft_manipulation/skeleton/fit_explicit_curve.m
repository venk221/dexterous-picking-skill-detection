function [curve, coefs, skel] = fit_explicit_curve(cur_joint_pos)
[ee_cart_pos, j2_cart_pos] = compute_cart_pos(cur_joint_pos);
skel = skeleton(j2_cart_pos, ee_cart_pos);
[curve, coefs] = fit_explicit(skel);
end

