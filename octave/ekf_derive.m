pkg load symbolic

% state
syms q0 q1 q2 q3
x = [q0; q1; q2; q3];

% measurement
syms ax ay az bx by bz
y1 = [ax; ay; az];
y2 = [bx; by; bz];

% f(x)
syms f wx wy wz bx by bz
omega = [wx; wy; wz]; %assume bias are canceled
f = [vpa(1/2) * [-q1 -q2 -q3; q0 -q3 q2; q3 q0 -q1; -q2 q1 q0]] * omega;

% P[t-1]
% Q: process noise
syms p_last_11 p_last_22 p_last_33 p_last_44 q11 q22 q33 q44
P_last = [p_last_11 0 0 0; 0 p_last_22 0 0; 0 0 p_last_33 0; 0 0 0 p_last_44];
Q = [q11 0 0 0; 0 q22 0 0; 0 0 q33 0; 0 0 0 q44];

% F
F = jacobian(f, x);

syms dt
P = P_last + dt * (F*P_last + P_last*F' + Q);

% R1: covariance matrix of acceleromter
syms r1_11 r1_22 r1_33
R1 = [r1_11 0 0; 0 r1_22 0; 0 0 r1_33];

% h1(x)
h1_11 = 2*(q1*q3 - q0*q2);
h1_21 = 2*(q2*q3 + q0*q1);
h1_31 = (q0^2-q1^2-q2^2+q3^2);
h1 = [h1_11; h1_21; h1_31];

% H1
H1 = jacobian(h1, x);

% K1: kalman gain of accelerometer
syms p1_11 p1_22 p1_33 p1_44
P1 = [p1_11 0 0 0; 0 p1_22 0 0; 0 0 p1_33 0; 0 0 0 p1_44];
K1 = P1*H1'*(H1*P1*H1' + R1)';

% R2: covariance matrix of magnetometer
syms r2_11 r2_22 r2_33
R2 = [r2_11 0 0; 0 r2_22 0; 0 0 r2_33];

% h2(x)
syms h2
h2_11 = 2*(q1*q2+q0*q3);
h2_21 = q0^2-q1^2-q2^2-q3^2;
h2_31 = 2*(q2*q3-q0*q1);
h2 = [h2_11; h2_21; h2_31];

% H2
H2 = jacobian(h2, x);

% K2: kalman gain of magnetometer
syms p2_11 p2_22 p2_33 p2_44
P2 = [p2_11 0 0 0; 0 p2_22 0 0; 0 0 p2_33 0; 0 0 0 p2_44];
K2 = P2*H2'*(H2*P2*H2' + R2)';
