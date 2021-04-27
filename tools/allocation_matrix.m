pkg load symbolic
syms d c

% allocation matrix for quadrotor
a1 = [1 1 1 1];
a2 = [-d +d +d -d];
a3 = [+d +d -d -d];
a4 = [-c +c -c +c];
disp("allocation matrix for quadrotor:")
A = [a1; a2; a3; a4]
disp("invese of allocation matrix for quadrotor")
inv(A)
