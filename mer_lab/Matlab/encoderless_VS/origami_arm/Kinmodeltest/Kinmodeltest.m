

[basex, basey, diffangle] = Findbase(88, 37);
filename = 'Reading_kintest.xlsx';
A = readmatrix(filename);
len1 = A(:,1);
len2 = A(:,2);
posx = A(:,4);
posy = A(:,5);

[obx, oby] = Transform(posx, posy, basex, basey, diffangle);
[calcx, calcy] = Calcpos(len1, len2);

scatter(obx,oby, 'red', 'filled');
hold on;
scatter(calcx,calcy, 'blue', 'filled');
legend('Observed','Calculated')
legend('Location','northwest')
