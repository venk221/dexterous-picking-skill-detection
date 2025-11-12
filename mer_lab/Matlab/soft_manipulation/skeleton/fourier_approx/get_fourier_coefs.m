function [curve, fourier_coefs] = get_fourier_coefs(cur_joint_pos)
    [ee_cart_pos, j2_cart_pos] = get_config(cur_joint_pos);
    skeleton = get_skeleton(j2_cart_pos, ee_cart_pos);
    [curve, fourier_coefs] = get_fourier_fit(skeleton);
end

