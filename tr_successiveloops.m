clc;
close all
clear

% longitudinal (theta)
tr_theta = 1.4;
%parameters
btheta = 1.152;
zeta = .7;
wn = pi/2/tr_theta/sqrt(1-zeta^2);
denom = [1 2*zeta*wn wn^2];
kd_theta = denom(2)/btheta;
kp_theta = denom(3)/btheta;
fprintf('kd_theta = %f\n',kd_theta)
fprintf('kp_theta = %f\n',kp_theta)
num = [kp_theta*btheta];
sys = tf(num,denom);
step(sys)
hold on

% 
% 
% inner loop
tr_phi = 0.3;
Jx = .0047;
wn = pi/2/tr_phi/sqrt(1-zeta^2);
denom = [1 2*zeta*wn wn^2];
kd_phi = denom(2)*Jx;
kp_phi = denom(3)*Jx;
fprintf('kd_phi = %f\n',kd_phi)
fprintf('kp_phi = %f\n',kp_phi)
num = [kp_phi/Jx];
sys = tf(num,denom);
figure(1)
step(sys)
hold on
figure(2)
rlocus(sys)


%outer loop yaw
M = 6;
tr_psi = tr_phi*M;
%
l1 = .85;
l2 = .3048;
m1 = .891;
m2 = 1.0;
Fe = (m1*l1-m2*l2)*9.81/l1;
Jz = .0041;
bpsi = l1*Fe/(m1*l1^2+m2*l2^2+Jz);
zeta = .7;
wn = pi/2/tr_psi/sqrt(1-zeta^2);
denom = [1 2*zeta*wn wn^2];

kd_psi = denom(2)/bpsi;
kp_psi = denom(3)/bpsi;
fprintf('kd_psi = %f\n',kd_psi)
fprintf('kp_psi = %f\n',kp_psi)
num = [bpsi*kp_psi];
sys = tf(num,denom);
figure(1)
step(sys)
% 
% % 
legend('\theta','\phi','\psi')
