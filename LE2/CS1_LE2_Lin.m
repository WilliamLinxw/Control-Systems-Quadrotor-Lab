%% Unstable zeros
sys = tf([-1 2000],[0.03 60 0 0]);

% For k >= 0
rlocus(sys)
title("Root locus for proportional controller", 'Fontsize', 16)

% Find a PD Controller
[C_pd, info] = pidtune(sys, 'PD');
%C_pd = tf([0.15 0.22],[1]);

% Closed loop system
tf_cl = minreal(feedback(C_pd*sys, 1));

% From transfer function to state space
[A, B, C, D] = tf2ss([-0.8665 1733 998.3], [1 1999 1733 998.3]);
sys_feedback = ss(A,B,C,D);

% Get the corresponding initial states for the state space
% Using \delta z(0) = y(0) = Cx(0); 
% z'(0) = y'(0) = Cx'(0) = CAx(0) + CBu(0), where u (0) = 0
% {z(0) = Cx(0); z'(0) = CAx(0)}, solve for x(0) 
% It's an underdetermined system, I used pseudo inverse for one x(0)
equations_matrix = [C*A; C];
initial_values = pinv(equations_matrix)*[0;-1];

% Response with initial values
initial(sys_feedback, initial_values, 4)
title("Response to initial conditions for 4 seconds, k_p=0.015, k_d=0.026", "Fontsize", 12)

initial(sys_feedback, initial_values, 4)
title("Response to initial conditions for 20 seconds, k_p=0.015, k_d=0.026", "Fontsize", 12)
%% Time Delay
k1 = 0.1;
k2 = 1;
k3 = 2;

tf_ol1 = tf(100*k1/3, [1 0.15 0]);
[Gm1,Pm1,Wcg1,Wcp1] = margin(tf_ol1);
max_time_delay1 = deg2rad(Pm1)/Wcp1;

tf_ol2 = tf(100*k2/3, [1 0.15 0]);
[Gm2,Pm2,Wcg2,Wcp2] = margin(tf_ol2);
max_time_delay2 = deg2rad(Pm2)/Wcp2;

tf_ol3 = tf(100*k3/3, [1 0.15 0]);
[Gm3,Pm3,Wcg3,Wcp3] = margin(tf_ol3);
max_time_delay3 = deg2rad(Pm3)/Wcp3;


delay = 1e-3;

tf_delay = tf(100/3,[1 0.15 0], 'InputDelay', delay);
feedback1 = feedback(k1*tf_delay, 1);
feedback2 = feedback(k2*tf_delay, 1);
feedback3 = feedback(k3*tf_delay, 1);
step(feedback1, 50)
hold on
step(feedback2, 50)
hold on
step(feedback3, 50)
title("Step reponse for k=0.1, k=1, k=2","Fontsize",16)

legend('k=0.1', 'k=1', 'k=2')

%% Nyquist plot
sys_ndnd = tf(100, [3 0 0]);
subplot(2,2,1)
nyqlog(sys_ndnd)
title("No drag no delay: No k_p>0 can asymptotically stabilize the system, it's always marginally stable",'fontsize',16)
[Gm1,Pm1,Wcg1,Wcp1] = margin(sys_ndnd);

sys_drag = tf(100, [3 0.45 0]);
subplot(2,2,2)
nyqlog(sys_drag)
title("Only drag: k_p > 0",'fontsize',16)
[Gm2,Pm2,Wcg2,Wcp2] = margin(sys_drag);
Gm2

sys_delay = tf([-0.1 200],[0.003 6 0 0]);
subplot(2,2,3)
nyqlog(sys_delay)
[Gm3,Pm3,Wcg3,Wcp3] = margin(sys_delay);
title("Only delay: No k_p>0 could stablize it",'fontsize',16)
Gm3

sys_dd = tf([-0.1 200], [0.003 6.00045 0.9 0]);
subplot(2,2,4)
nyqlog(sys_dd)
[Gm4,Pm4,Wcg4,Wcp4] = margin(sys_dd);
title("Drag + delay: 0 < k_p < 4.5002",'fontsize',16)
Gm4
%% PID Control
p1 = 1;
p2 = 2;
p3 = 4;

t1 = 1;
t2 = 2;
t3 = 4;

pi_ol1 = tf([1 p1], [0.03 0 0 0]);
pi_ol2 = tf([1 p2], [0.03 0 0 0]);
pi_ol3 = tf([1 p3], [0.03 0 0 0]);

% subplot(1,3,1)
% rlocus(pi_ol1)
% title('PI Control with pi = 1','fontsize',16)
% 
% subplot(1,3,2)
% rlocus(pi_ol2)
% title('PI Control with pi = 2','fontsize',16)
% 
% subplot(1,3,3)
% rlocus(pi_ol3)
% title('PI Control with pi = 4','fontsize',16)

pd_ol1 = tf([t1 1], [0.03 0 0]);
pd_ol2 = tf([t2 1], [0.03 0 0]);
pd_ol3 = tf([t3 1], [0.03 0 0]);

subplot(1,3,1)
rlocus(pd_ol1)
title('PD Control with T_d = 1','fontsize',16)

subplot(1,3,2)
rlocus(pd_ol2)
title('PD Control with T_d = 2','fontsize',16)

subplot(1,3,3)
rlocus(pd_ol3)
title('PD Control with T_d = 4','fontsize',16)

%% Bode Plot
t1 = 1;
t2 = 2;
t3 = 4;

pd_ol_drag1 = tf([t1 1],[1 0.15 0]);
pd_ol_drag2 = tf([t2 1],[1 0.15 0]);
pd_ol_drag3 = tf([t3 1],[1 0.15 0]);

subplot(1,3,1)
bode(pd_ol_drag1)
title('PD control with drag for T_d = 1','fontsize',16)
[Gm1,Pm1,Wcg1,Wcp1] = margin(pd_ol_drag1);

subplot(1,3,2)
bode(pd_ol_drag2)
title('PD control with drag for T_d = 2','fontsize',16)
[Gm2,Pm2,Wcg2,Wcp2] = margin(pd_ol_drag2);

subplot(1,3,3)
bode(pd_ol_drag3)
title('PD control with drag for T_d = 4','fontsize',16)
[Gm3,Pm3,Wcg3,Wcp3] = margin(pd_ol_drag3);

subplot(1,3,1)
margin(pd_ol_drag1);
subplot(1,3,2)
margin(pd_ol_drag2);
subplot(1,3,3)
margin(pd_ol_drag3);


