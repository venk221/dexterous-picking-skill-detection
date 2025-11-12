function [posx3, posy3] = Calcpos(len1, len2)
    base1 = 11;
    base2 = 2;
    markerh = 37;
    tlength = 68;

    posx1 = zeros(length(len1),1);
    posy1 = base1 * ones(length(len1),1);
    x = 45;
    len1 = len1 + x;
    len2 = len2 + x;
    len3 = len1;
    d = tlength /2 /cosd(30);
    kappa = 2 * sqrt(len1.^2 + len2.^2 + len3.^2 - len1.*len2 - len2.*len3 - len3.*len1)./(d*(len1+len2+len3));
    l = (len1 + len2 + len3)/3;
    
    theta = l .* kappa;
    
    posy2 = posy1;
    posx2 = posx1;
    for i = 1:length(len1)
        if len1(i) == len2(i)
            posy2(i) = posy1(i) + l(i);
            posx2(i) = posx1(i);
        else
            posy2(i) = posy1(i) + sin(theta(i)) / kappa(i);
            posx2(i) = posx1(i) + (1 - cos(theta(i))) / (kappa(i) * sign(len2(i) - len1(i)));
        end
    end
    
    posy3 = posy2 + (markerh + base2) * cos(theta);
    posx3 = posx2 + (markerh + base2) * sin(theta) .* sign(len2 - len1);
    