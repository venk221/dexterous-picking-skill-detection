function [obangle, posx3, posy3] = Transform(basex, basey, diffangle)

    load('curval.mat', 'res', 'anglecur', 'posx', 'posy');
    %Swapping axes as well
    posx2 = posy - basey;
    posy2 = posx - basex;
    
    T = [cosd(diffangle) -sind(diffangle); sind(diffangle) cosd(diffangle)];
    
    posx3 = T(1,1) * posx2 + T(1,2) * posy2;
    posy3 = T(2,1) * posx2 + T(2,2) * posy2;
    obangle = anglecur;
%     p = [posx3 posy3 obangle];
%     disp(p);