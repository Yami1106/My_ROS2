% Controller 

J = 3; %given inertia
B = 1; % given damping force 

zeta = 1 % system is critically damped
Ts = 2 % settling time is given 

wn = 4/(zeta * Ts) % natural frequency 

Kp = (J * wn^2) % Proportional constant
Kd =  2*(zeta*wn*J) - B % differential constant 

d = 0.8 % given disturbance 

Ki = 4.8; % integral constant


