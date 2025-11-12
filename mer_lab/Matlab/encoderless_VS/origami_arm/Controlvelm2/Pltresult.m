
A = ES(:);
B = cell2mat(A);
B = B(:,:);
ty = -B(:,1);
tx = -B(:,2);
py = -B(:,3);
px = -B(:,4);

scatter(tx,ty, 'red', 'filled');
hold on;
scatter(px,py, 'blue');
line(px,py);
legend('Target','Path')
legend('Location','northeast')