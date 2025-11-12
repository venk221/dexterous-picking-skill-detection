function [] = Tendonlen2(obx, oby)
    base1 = 11;
    base2 = 2;
    markerh = 37;
    tlength = 68;
    d = tlength /2 /cosd(30);
    
    posy1 = oby;
    posx1 = obx;
    
    m = markerh + base2;
    posy2 = posy1 - base1;
    posx2 = posx1;
    
    if abs(posx1) < 1.6
        clen1 = posy2-m; clen2 = posy2-m;
    else
        kappa = 2 * posx2 / (posx2^2 + posy2^2 - m^2);
        r = abs(1 / kappa);
        theta = acos(abs(r^2 + m*posy2 - abs(r*posx2))/(m^2 + r^2));
        l = abs(r * theta);
        theta = theta * sign(posx2) * -1;
        clen1 = l + d * cosd(60) * theta;
        clen2 = l - d * theta;
    end
    
    save('tlen2.mat', 'clen1', 'clen2');
%     p = [clen1 clen2];
%     disp(p);
%     
    
end