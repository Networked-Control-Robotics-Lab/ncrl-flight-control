%input file name: compass.csv
%csv format for this script: mx,my,mz\n

csv = csvread("compass.csv");
mx = csv(:, 1);
my = csv(:, 2);
mz = csv(:, 3);

scatter3(mx,my,mz)

xlim([-100, 100])
ylim([-100, 100])
zlim([-100, 100])
xlabel('mx (uT)')
ylabel('my (uT)')
zlabel('mz (uT)')
daspect([1 1 1]) %set aspect ratio to 1:1:1
grid on
disp('press any key to leave')
pause
