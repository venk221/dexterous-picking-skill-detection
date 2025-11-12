function readSerialData(src,~)
    data = readline(src);
    %disp(data);
    r = split(data);
    r = processData(r);
    %disp(r);
    %Initialize(r, 50, 0);
    Track(r);
    %[basex, basey, diffangle] = Findbase(87, 37);
    basex = -1.238819794761504e+02; basey = -5.408804033301664; diffangle = 2.5;
    [obangle, obx, oby] = Transform(basex, basey, diffangle);
    Tendonlen(obangle, obx, oby);
    
end