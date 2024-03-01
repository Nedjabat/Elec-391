J = 6.4E-7;     % Rotor Inertia
B = 1.436E-3;   % Kinetic friction
Kt = 0.412;     % Torque constant
Kv = 0.308;     % Back EMF constant
R = 43.36;      % Resistance
L = 2.904E-3;   % Reluctance
s = tf('s');
P_motor = Kt/(s*((J*s+B)*(L*s+R)+Kt*Kv));  % Open-loop transfer function, 

t = 0:0.001:3;
step(P_motor,t)       % motor open-loop step response with output theta (rad)

%isstable(P_motor)     % returns 1 if system is stable, 0 if not
%pole(P_motor)         % returns poles of the transfer function

%sys_cl = feedback(P_motor,1);    % closed-loop transfer function with C(s) = 1

%step(sys_cl,t)  % motor closed-loop step response

%pzmap(sys_cl)   % pole-zero mapping
%damp(sys_cl)    % damping and natural frquencies of the poles

%{
%Proportional Control
Kp = 1;

for i = 1:3
    C(:,:,i) = pid(Kp);
    Kp = Kp + 30;
end
sys_cl = feedback(C*P_motor,1);

t = 0:0.001:0.2;
step(sys_cl(:,:,1), sys_cl(:,:,2), sys_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Response to a Step Reference with Different Values of K_p')
legend('Kp = 1',  'Kp = 31',  'Kp = 61')


%Disturbance based on Kp
dist_cl = feedback(P_motor,C);
step(dist_cl(:,:,1), dist_cl(:,:,2), dist_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Response to a Step Disturbance with Different Values of K_p')
legend('Kp = 1', 'Kp = 31','Kp = 61')
%}

%{
%Integral Control given Kp = 60
Kp = 60;
Ki = 0.01;
for i = 1:3
    C(:,:,i) = pid(Kp,Ki);
    Ki = Ki + 300;
end

sys_cl = feedback(C*P_motor,1);
t = 0:0.001:0.2;
step(sys_cl(:,:,1), sys_cl(:,:,2), sys_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Response to a Step Reference with K_p = 60 and Different Values of K_i')
legend('Ki = 0.01', 'Ki = 1.01', 'Ki = 2.01')
%}

%{
%Disturbance based on Ki
dist_cl = feedback(P_motor,C);
step(dist_cl(:,:,1), dist_cl(:,:,2), dist_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Response to a Step Disturbance with K_p = 21 and Different Values of K_i')
legend('Ki = 0.01',  'Ki = 11',  'Ki = 2.01')
%}

%{
%Derivative control given Kp = 60, Ki = 600
Kp = 60;
Ki = 600;
Kd = 0;

for i = 1:3
    C(:,:,i) = pid(Kp,Ki,Kd);
    Kd = Kd + 0.1;
end

sys_cl = feedback(C*P_motor,1);
t = 0:0.001:0.1;
step(sys_cl(:,:,1), sys_cl(:,:,2), sys_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Step Response with K_p = 60, K_i = 600 and Different Values of K_d')
legend('Kd = 0', 'Kd = 0.10', 'Kd = 0.2')
%}

%{
%Disturbance based on Kd
dist_cl = feedback(P_motor,C);
t = 0:0.001:0.2;
step(dist_cl(:,:,1), dist_cl(:,:,2), dist_cl(:,:,3), t)
ylabel('Position, \theta (radians)')
title('Step Response with K_p = 21, K_i = 500 and Different values of K_d')
legend('Kd = 0', 'Kd = 0.10', 'Kd = 0.20')
%}

%stepinfo(sys_cl(:,:,2)) % step response results