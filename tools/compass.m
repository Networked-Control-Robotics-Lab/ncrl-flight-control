%input file name: compass.csv
%csv format for this script: mx,my,mz\n

%read data from csv
csv = csvread("compass.csv");
mx = csv(:, 1);
my = csv(:, 2);
mz = csv(:, 3);

%normalization
norms = sqrt(mx(:) .* mx(:) + my(:) .* my(:) + mz(:) .* mz(:));
mx_normalized = mx(:) ./ norms(:)
my_normalized = my(:) ./ norms(:)
mz_normalized = mz(:) ./ norms(:)

%raw data
figure(1);
scatter3(mx,my,mz)
%
xlim([-100, 100])
ylim([-100, 100])
zlim([-100, 100])
xlabel('mx (uT)')
ylabel('my (uT)')
zlabel('mz (uT)')
daspect([1 1 1]) %set aspect ratio to 1:1:1
grid on

%normalized data
figure(2);
scatter3(mx_normalized, my_normalized, mz_normalized)
%
xlim([-1, 1])
ylim([-1, 1])
zlim([-1, 1])
xlabel('mx')
ylabel('my')
zlabel('mz')
daspect([1 1 1]) %set aspect ratio to 1:1:1
grid on
disp('press any key to leave')
pause
