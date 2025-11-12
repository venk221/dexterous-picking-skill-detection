function [posx3, posy3] = Transform(posx, posy, basex, basey, diffangle)
    %Swapping axes as well
    posx2 = posy/1.8 * 1.67 - basey;
    posy2 = posx/1.8 * 1.67 - basex;
    
    T = [cosd(diffangle) -sind(diffangle); sind(diffangle) cosd(diffangle)];
    
    posx3 = T(1,1) * posx2 + T(1,2) * posy2;
    posy3 = T(2,1) * posx2 + T(2,2) * posy2;