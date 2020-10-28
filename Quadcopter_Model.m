%% Quadcopter Mathematical Modelling : Plant Model
% Siddhartha Devineni

%% Kinematics : Linear position and Linear velocity in inertial frame
% Linear position
lp = [x;y;z];
% Linear velocity
syms t
lv = diff(p,t);
% Attitude : Angular position and angular velocity
% Angular position
syms eta
eta = [phi;teta;sci];
% Angular velocity
vi = diff(eta,t);

%% Kinematics in body frame
% Linear velocity
vb = [v_x;v_y;v_z];
%angular velocity
av = [p;q;r];
% Relation between the angular velocity and the roll, picth and yaw rates :
syms theta phi sci
av = [1 0 -sin(teta);0 cos(phi) sin(phi)*cos(theta);0 -sin(theta) cos(phi)*cos(theta)]*[diff(phi,t); diff(theta,t); diff(sci,t)];

%% Forces (thrust) and Torque
% The thrusts f1,f2,f3,f4 produced by the 1st,2nd,3rd,4th rotors with w1,w2,w3,w4 as their angular velocities 
f1 = k*w1^2;
f2 = k*w2^2;
f3 = k*w3^2;
f4 = k*w4^2;
% The combined thrust F in the body frame :
F = f1 + f2 + f3 + f4;
% Total thrust along the z-axis in the body frame :
Tb = [0;0;F];
% Torque Tm along the rotor axes because of angular velocity and acceleration
% of the rotor
Tm1 = d*w1^2;
Tm2 = d*w2^2; 
Tm3 = d*w3^2;
Tm4 = d*w4^2;
% Total torque Taub in body frame wiht length l
Taub = [l*k*(w4^2 - w2^2); l*k*(w3^2 - w1^2); d*(-w1^2 + w2^2 - w3^2 + w4^2)];
Taub = [Tau_phi; Tau_theta; Tau_sci];

%% Equations of motions
Eq = [0; 0; -g] + (F/m)*[cos(phi)*sin(theta)*cos(sci) + sin(phi)*sin(sci);cos(phi)*sin(theta)*sin(sci) - sin(phi)*cos(sci); cos(phi)*cos(theta) ]...
    .. - (1/m)*[Ax 0 0 ; 0 Ay 0; 0 0 Az]*[dx; dy; dz];

%% Total external torque 
Imat*d(av) + cross(Imat,av) = Taub;


