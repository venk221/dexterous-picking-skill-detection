function [f, coefs] = get_fourier_fit(skeletonPts)
    curve_type = 'fourier2';
    curvePtsX = skeletonPts(:,1);
    curvePtsY = skeletonPts(:,2);
    f = fit(curvePtsX, curvePtsY, curve_type);
    coefs = coeffvalues(f);
    %[f.a0, f.a1, f.b1, f.a2, f.b2, f.w];
end