function y_i = I_c_i_(x_i, i, c, varphi)
    % projection function from R3 to R2, input takes camera pose and focal
    % length (c and varphi respectively)

    x_c = (R_(c(:,i))') * (x_i - j_(c(:,i)));
    y_i = varphi(i) * [x_c(1)/x_c(3); x_c(2)/x_c(3)];
end