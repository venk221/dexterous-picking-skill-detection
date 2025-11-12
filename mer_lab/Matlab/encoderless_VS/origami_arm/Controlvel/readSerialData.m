function readSerialData(src,~)
    data = readline(src);
    %disp(data);
    r = split(data);
    r = processData(r);
    %p = [r(1),r(2), r(3), r(4)];
    %disp(p);
    %Initialize(r, 50, 0);
    Track(r);
%     [basex, basey, diffangle] = Findbase(88, 37);
    basex = -125; basey = 0; diffangle = 0;
    [obangle, obx, oby] = Transform(basex, basey, diffangle);
%     Tendonlen(obangle, obx, oby);
    Tendonlen2(obx, oby);
end