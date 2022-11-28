%% Lead Compensator
G_CF = tf(9.81, [1 2/3 0 0]);

K = tf([40/6000 1/6000], [2 1]);

margin(G_CF)

margin(K*G_CF)

step(feedback(K*G_CF,1))

%% Cascade Control
g_theta = tf(1,[1 0]);
tf_theta = feedback(5*g_theta,1);

G_no_theta = tf(9.81, [1 2/3 0]);

G_out = tf_theta * G_no_theta;

% Crossover @>=4rad/s
K1 = tf([1 1],[1*(7-4*sqrt(3)) 1]);
% bode(K1*G_out);
% step(feedback(K1*G_out,1));
% stepinfo(feedback(K1*G_out,1));


% Phase margin >= 60 degree
K2 = tf([sqrt(3)/20 1/20],[1/3*sqrt(3) 1]);
% bode(K2*G_out);
% step(feedback(K2*G_out,1));
% stepinfo(feedback(K1*G_out,1));
margin(K1*G_out)
hold on
margin(K2*G_out)
hold off
title("Phase Margin_1=14.2^{\circ}, w_{c1}=5.89rad/s       Phase Margin_2=62.9^{\circ}, w_{c2}=0.729rad/s")

step(feedback(K1*G_out,1));
hold on
step(feedback(K2*G_out,1));
legend('K_{out,1}','K_{out,2}')

stepinfo(feedback(K*G_CF,1))
stepinfo(feedback(K1*G_out,1))
stepinfo(feedback(K2*G_out,1))


%% Model Mismatch
% Max delay
[Gm1,Pm1,Wcg1,Wcp1] = margin(K1*G_out);
max_time_delay1 = deg2rad(Pm1)/Wcp1;

[Gm2,Pm2,Wcg2,Wcp2] = margin(K2*G_out);
max_time_delay2 = deg2rad(Pm2)/Wcp2;

% Step response with delay
tf_delay = tf(1,1,'InputDelay',0.2);
step(feedback((K1*G_out),tf_delay),10)
ylim([-5,5])
hold on

step(feedback((K2*G_out),tf_delay),10)
ylim([-5,5])
hold off
title("Step response with delay")
legend('K_{out,1}','K_{out,2}')

%S error
syms x
S1_num = sym2poly(49.05*((7-4*sqrt(3))*x+1));
S1_den = sym2poly(x*((7-4*sqrt(3))*x+1)*(x+2/3)*(x+5) +49.05*(x+1));
S1 = tf(S1_num, S1_den);

S2_num = sym2poly(49.05*(sqrt(3)/3*x+1));
S2_den = sym2poly(x*(sqrt(3)/3*x+1)*(x+2/3)*(x+5)+(49.05/20)*(sqrt(3)*x+1));
S2 = tf(S2_num, S2_den);

%Step response with error
T1 = feedback(K1*G_out,1);
tf_error_1 = T1 - pi/180*S1;

T2 = feedback(K2*G_out,1);
tf_error_2 = T2 - pi/180*S2;

% Lag Compensator
K_lag = tf([400 40],[400 1]);

S_lag_num = sym2poly(49.05*(sqrt(3)/3*x+1)*(400*x+1));
S_lag_den = sym2poly((sqrt(3)/3*x+1)*(400*x+1)*x*(x+2/3)*(x+5)+(49.05/20)*(40)*(sqrt(3)*x+1)*(10*x+1));
S_lag = tf(S_lag_num, S_lag_den);

T_lag = feedback(K_lag*K2*G_out,1);
tf_error_lag = T_lag - pi/180*S_lag;

step(tf_error_1,30)
hold on
step(tf_error_2,30)
hold on
step(tf_error_lag,30)
title("Step Response with measurement error")
legend('K_{out,1}','K_{out,2}','K_{lag}*K_{out,2}')
hold off

margin(K_lag*K2*G_out)