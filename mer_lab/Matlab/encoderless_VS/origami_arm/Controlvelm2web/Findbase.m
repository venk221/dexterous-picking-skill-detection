function [basex, basey, diffangle] = Findbase(a,b)
    %Finds base wrt initialization position
    load('initval.mat','res', 'angle', 'pixelx', 'pixely');
    
    diffangle = 90 - angle;
    dist = a + b;
    basex = - dist * cosd(diffangle);
    basey = - dist * sind(diffangle);
end