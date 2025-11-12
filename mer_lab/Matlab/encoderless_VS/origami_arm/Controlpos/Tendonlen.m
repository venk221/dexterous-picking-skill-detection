function [] = Tendonlen(obangle, obx, oby)
    base1 = 11;
    base2 = 2;
    markerh = 37;
    tlength = 68;
    d = tlength /2 /cosd(30);
    
    obangler = deg2rad( obangle );
    posy1 = oby - (markerh + base2) * cos(obangler);
    posx1 = obx + (markerh + base2) * sin(obangler);
    
    posy2 = posy1 - base1;
    posx2 = posx1;
    
    if abs(posx1) < 1.5
        clen1 = posy2; clen2 = posy2;
    else
        kappa = 2 * posx2 / (posx2^2 + posy2^2);
        l = abs(obangler / kappa);
        clen1 = l + d * cosd(60) * obangler;
        clen2 = l - d * obangler;
    end
    save('tlen.mat', 'clen1', 'clen2');
%     p = [clen1 clen2];
%     disp(p);
end