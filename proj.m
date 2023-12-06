clear 
close all
clc

matlab_graphics
% LTI system 

Yv_nom = -0.1068;
unc_Yv = 3 * 4.26;

Yp_nom = 0.1192;
unc_Yp = 3 * 2.03;

Lv_nom = -5.9755;
unc_Lv = 3 * 1.83;

Lp_nom = -2.6478;
unc_Lp = 3 * 2.01;

Yd_nom = -10.1647;
unc_Yd = 3 * 1.37;

Ld_nom = 450.7085;
unc_Ld = 3 * 0.81;

g = 9.81;

% Stability Derivatives
Yv = ureal('Yv',Yv_nom,'Perc',unc_Yv);
Yp = ureal('Yp',Yp_nom,'Perc',unc_Yp);
Lv = ureal('Lv',Lv_nom,'Perc',unc_Lv);
Lp = ureal('Lp',Lp_nom,'Perc',unc_Lp);

% Control Derivatives
Yd = ureal('Yd',Yd_nom,'Perc',unc_Yd);
Ld = ureal('Ld',Ld_nom,'Perc',unc_Ld);
          
A = [ Yv    Yp     g;
      Lv    Lp     0;
      0      1      0];
B = [Yd
      Ld
      0];
C = [0   1   0;
     0   0   1
     Yv   Yp  0];
D = [0 
     0 
     Yd];
%Uncertain Plant
sys_un = ss(A,B,C,D);
sys_un.u = '\delta_{lat2}';
sys_un.y = {'p','\phi','ay'};

%Nominal Plant 
sys_nom = sys_un.NominalValue;
sys_nom.u = '\delta_{lat2}';
sys_nom.y = {'p','\phi','ay'};

%Transfer function
%First output
G_delta_lat_p_nom = tf(sys_un(1));

figure
pzplot(sys_un(1));
hold on 
pzplot(sys_nom(1));
grid on                               
legend('Uncertain model','Nominal model','location','east')

z_delta_lat_p = tzero(sys_nom(1));
p_delta_lat_p = pole(sys_nom(1));

%Second output
G_delta_lat_phi_nom = tf(sys_un(2));

figure
pzplot(sys_un(2));
hold on 
pzplot(sys_nom(2));
grid on                               
legend('Uncertain model','Nominal model','location','east')

z_delta_lat_phi = tzero(sys_nom(2));
p_delta_lat_phi = pole(sys_nom(2));

%Third output
G_delta_lat_p_nom = tf(sys_un(3));

figure
pzplot(sys_un(3));
hold on 
pzplot(sys_nom(3));
grid on                               
legend('Uncertain model','Nominal model','location','east')

z_delta_lat_ay = tzero(sys_nom(3));
p_delta_lat_ay = pole(sys_nom(3));


figure 
bode(sys_un)
hold on
bode(sys_nom)
grid on
legend('Uncertain model','Nominal model','location','northwest')

%Step Response
figure 
step(sys_un)
hold on
step(sys_nom)
grid on
legend('Uncertain model','Nominal model','location','northwest')

%P controller (R_phi)
R_phi = tunablePID('R_phi','P')
R_phi.u = 'e_\phi';
R_phi.y = 'p_0';

%PID2 Controller (R_p)
R_p = tunablePID2('R_p','PID')
R_p.c.Value = 0;
R_p.c.Free = false;
R_p.b.Value = 1;
R_p.b.Free = false;
R_p.Tf.Value = 0.01;
R_p.Tf.Free = false;
R_p.u = {'p_0','p'};
R_p.y = {'\delta_{lat}'};

%Sum
SumOuter = sumblk('e_\phi = \phi_0 - \phi');

%Motor dynamic
gain = pid(0.99);
gain.u = '\delta_{lat}';
gain.y = '\delta_{lat1}';

motor = ss(tf( 1 , [0.02 1]));
motor.u = '\delta_{lat1}';
motor.y = '\delta_{lat2}';

%Connection
CL_tun = connect(SumOuter,R_phi,R_p,gain,motor,sys_nom,'\phi_0',{'\phi','p'})

%Nominal performance
omega_n = 13;       % [rad/s]
xi = 0.9;

s = tf('s');
%Complementary 
F_2 = omega_n^2/(s^2 + 2*s*xi*omega_n + omega_n^2);

%sensitivity
S_2 = 1-F_2;

figure
step(F_2)
grid on

stepinfo(F_2)

%Bode Sensitivity
figure
bodemag(F_2)
hold on
bodemag(S_2)
grid on
legend('$F\_2$','$S\_2$','interpreter','latex')
%%
M = 1.43;
Ac = 1e-3;
omega_b = 0.45*omega_n;

W_p = (s/M + omega_b)/(s + Ac*omega_b);
W_p2inv = makeweight(1e-3,0.5*10,1.4,0,1);
W_p2 = 1/W_p2inv;

figure
bodemag(F_2)
hold on
bodemag(S_2)
grid on
bodemag(1/W_p)
%bodemag(1/W_p2)
legend('$F\_2$','$S\_2$','$1/W\_p$','interpreter','latex', 'location','southeast')


%Control effort limitation
alpha = deg2rad(20)/5;
w_tau = 1/0.01;

W_q = alpha*(s+w_tau*1e-3)/(s+w_tau);

figure
bodemag(1/W_q)
grid on

%H_inf synthesis
W_p.u = 'e_\phi';
W_p.y = 'z_1';

W_q.u = '\delta_{lat}';
W_q.y = 'z_2';

%Connect
CL0 = connect(R_p, R_phi,gain,motor, sys_nom, SumOuter, W_p, W_q,...
    '\phi_0',{'z_1','z_2'},{'\delta_{lat2}','e_\phi','\phi','\delta_{lat}','\delta_{lat1}'});

opt = hinfstructOptions('Display','final','RandomStart',20);

[CL,gamma,info] = hinfstruct(CL0,opt);
showTunable(CL)

%Complementary sensitivity
F = getIOTransfer(CL,'\phi_0','\phi');

figure
bodemag(F)
grid on
 
%Step response
figure
step(F)
hold on
step(F_2)
grid on
legend('hinfstruct','second order', 'location','southeast')

Finfo = stepinfo(F)
F2info = stepinfo(F_2)

%Sensitivity
S = getIOTransfer(CL,'\phi_0','e_\phi');

figure
bodemag(S)
grid on
hold on
bodemag(1/W_p)
legend('$S$','$1/W\_p$','interpreter','latex', 'location','southeast')

%Control sensitivity
Q = getIOTransfer(CL,'\phi_0','\delta_{lat2}');

figure
bodemag(Q)
grid on
hold on
bodemag(1/W_q)
legend('$Q$','$1/W\_q$','interpreter','latex', 'location','southeast')

%Loop transfer function
L_f = (1-S)/S;

figure
margin(L_f)
grid on

%Linear simulation
t = linspace(0,6,10^4);
u = 0*(t<=1) + 10*(t>1 & t<= 3) - 10*(t>3 & t<= 5) + 0*(t>5);
u = deg2rad(u);

figure
delta_lat = lsim(Q,u,t);
lsimplot(Q,u,t);
hold on
grid on

%Phi simulation
figure
phi = lsim(F,u,t);
lsimplot(F,u,t);
hold on
grid on

%Robust stability
sys_un_array = usample(sys_un(2),60);

[~,Info] = ucover(sys_un_array,sys_nom(2),3);

W = Info.W1;

figure
bodemag(W,'r')
hold on
bodemag((sys_nom(2)-sys_un_array)/sys_nom(2))
grid on


figure
bodemag(F,'r',1/W,'g')
grid on
legend('$F$','$1/W$','interpreter','latex', 'location','southeast')

%Robust verification
w = logspace(-5,5,500);

FW = bode(F*W,w); 
SW = bode(S*W_p,w);

figure
semilogx(w, 20*log10(squeeze(FW)) + 20*log10(squeeze(SW)))
hold on 
semilogx(w,20*log10(w./w))
grid on
legend('$|W_p S|+|W F|$','$1$','interpreter','latex', 'location','southeast')



Tf_1 = tf(sys_nom);     % From input "\delta_{lat}" to output [y]
Tf_2=tf(Q);            %From input "\phi_0" to output "\delta_{lat}"
Tf_tot=Tf_1*Tf_2;


%% 
% Definizione delle matrici del sistema
A=double(A);
B=double(B);
C=double(C);
D=double(D);
%Cobs(:,3) = 0 ;

% Progettazione dell'osservatore di stato (L)
desiredEigenvalues = [-60 -50 -0];

% Calcolo della matrice dell'osservatore L utilizzando la funzione 'place'
L = place(A', C', desiredEigenvalues)';

Aobs = A - L*C;

% Simulazione del sistema con l'osservatore di stato
tspan = t; % Intervallo di tempo della simulazione

   
% Simulazione a passi di tempo fissi
numSteps = length(t);
dt = t(end)/length(t);
x_hat = zeros(3, numSteps);
x_hat(:, 1) = [0.4; 0.5; 0.1]; % Condizioni iniziali della stima dell'osservatore
y=lsim(Tf_tot,u,t)';
% y(2,:) = 0;
% y(3,:) = 0;
for k = 1:numSteps-1
    

    % Aggiornamento dell'osservatore di stato
    x_hat_dot = Aobs * x_hat(:, k) + B * delta_lat(k) + L * y(:,k) - L * D * delta_lat(k);
    x_hat(:, k+1) = x_hat(:, k) + x_hat_dot * dt;
end

x_hat=x_hat';
% Plot dei risultati
figure;
plot(tspan, phi, 'b', tspan, x_hat(:, 3), 'r--');
xlabel('Tempo');
ylabel('\phi');
legend('Stato reale', 'Stima dell osservatore');

figure;
plot(tspan, y(1,:), 'b', tspan, x_hat(:, 2), 'r--');
xlabel('Tempo');
ylabel('P');
legend('Stato reale', 'Stima dell osservatore');

%%
%Obs block
Bobs = [B - L*D L];
%  Bobs(:,1:2) = 0;
%  Bobs(:,4) = 0;
Bobs(:,3) = 0;
Obs = ss(Aobs,Bobs,eye(3),zeros(3,4));
Obs.u = {'\delta_{lat2}','p','\phi','ay'};
Obs.y = {'v_{est}','p_{est}','\phi_{est0}'};
 K = pid(1);
 K.u = '\phi_{est0}';
 K.y = '\phi_{est}';
sum = sumblk('e_\phi = \phi_0 - \phi_{est}');

%creo dei PID con i valori precedenti
R_p_fix=pid2(CL.Blocks.R_p);
R_phi_fix=pid(CL.Blocks.R_phi);
R_p_fix.Name='R_p_fix';
R_phi_fix.Name='R_phi_fix';

R_phi_fix.u = 'e_\phi';
R_phi_fix.y = 'p_0';

R_p_fix.u = {'p_0','p'};
R_p_fix.y = {'\delta_{lat}'};

Fullsys = connect(K,sum,R_phi_fix,R_p_fix,gain,motor,sys_nom,Obs,'\phi_0',{'p','\phi','ay'},{'\delta_{lat2}','\phi_{est}','e_\phi'});

%Complementary sensitivity
F_full = getIOTransfer(Fullsys,'\phi_0','\phi');

figure
bodemag(F_full)
grid on


figure
lsim(F_full,u,t)
grid on
hold on
lsim (F,u,t)
legend('$Con$ $observer$','$Senza$ $observer$','interpreter','latex', 'location','northeast')
%Step response
figure
step(F_full)
hold on
step(F_2)
grid on
legend('Sys + Obs','second order', 'location','southeast')

F_full_info = stepinfo(F_full)
F2info = stepinfo(F_2)

%Sensitivity
S_full = getIOTransfer(Fullsys,'\phi_0','e_\phi');

figure
bodemag(S_full)
grid on
hold on
bodemag(1/W_p)
hold on
legend('$S$','$1/W\_p$','interpreter','latex', 'location','southeast')

%Control sensitivity
Q_full = getIOTransfer(Fullsys,'\phi_0','\delta_{lat2}');

figure
bodemag(Q_full)
grid on
hold on
bodemag(1/W_q)
legend('$Q$','$1/W\_q$','interpreter','latex', 'location','southeast')

%Loop transfer function
L_full = (1-S_full)/S_full;

figure
subplot(1,2,1)
margin(L_full)
grid on
subplot(1,2,2)
margin(L_f)
grid on
legend('Sys + Obs','Sys');


figure
lsim(Q_full,u,t);
hold on
grid on

%%
%Robust stability
sys_un_array = usample(sys_un(2),200);

[~,Info] = ucover(sys_un_array,sys_nom(2),3);

W = Info.W1;

figure
bodemag(W,'r')
hold on
bodemag((sys_nom(2)-sys_un_array)/sys_nom(2))
grid on


figure
bodemag(F_full,'r',1/W,'g')
grid on
legend('$F_full$','$1/W$','interpreter','latex', 'location','southeast')

%Robust verification
w = logspace(-5,5,500);

FW_full = bode(F_full*W,w); 
SW_full = bode(S_full*W_p,w);

figure


semilogx(w, 20*log10(squeeze(FW)) + 20*log10(squeeze(SW)))
hold on 
semilogx(w, 20*log10(squeeze(FW_full)) + 20*log10(squeeze(SW_full)))
hold on
semilogx(w,20*log10(w./w))
grid on

legend('$|W_p S|+|W F|$','$|W_p S|+|W F|$ + Obs','$1$','interpreter','latex', 'location','southeast')

%%
%Montecarlo analysis
tic
num_sim = 100;
sys_mc = usample(sys_un(1:3),num_sim);
k = 0.75 + 0.25 * rand(num_sim,1);
x = linspace(0,num_sim,num_sim);
delta_sum = 0;
over_sum = 0;
set_sum = 0;

parfor i = 1 : num_sim
   
    warning('off','all');

    sys = sys_mc(:,:,i,1);
    ki = k(i);
    gain = pid(ki);
    gain.u = '\delta_{lat}';
    gain.y = '\delta_{lat1}';

    sys_full = connect(K,sum,R_phi_fix,R_p_fix,gain,motor,sys,Obs,'\phi_0',{'p','\phi','ay'},{'\delta_{lat2}','\phi_{est}','e_\phi','\delta_{lat2}'});
    
    F_sys = getIOTransfer(sys_full,'\phi_0','\phi');
    Q_sys = getIOTransfer(sys_full,'\phi_0','\delta_{lat2}');


    %Step response
    F_info(i) = stepinfo(F_sys);
    over(i) = F_info(i).Overshoot;
    set(i) = F_info(i).SettlingTime;


    %Doublet response
    delta(:,i) = lsim(Q_sys,u,t)';
    delta_max(i) = max(abs(delta(:,i)));

    %sum update
    delta_sum = delta_sum + delta_max(i);
    set_sum = set_sum + set(i);
    over_sum = over_sum + over(i);


end

%Mean value
delta_mean = delta_sum / num_sim
set_mean = set_sum / num_sim
over_mean = over_sum / num_sim

%STD
sigma_delta = std(delta_max)
sigma_over = std(over)
sigma_set = std(set)


figure
plot( x , delta_max ,'o', x , delta_mean * ones(num_sim,1) );
legend('\delta_{max}','\delta_{mean}');
title('\delta_{lat}');

figure
plot( x , over ,'o', x , over_mean * ones(num_sim,1) );
legend('Overshoot','Mean Overshoot');
title('Percentage Overshoot');

figure
plot( x , set ,'o', x , set_mean * ones(num_sim,1) );
legend('Settling time','Mean Settling time');
title('Settling time');

figure
histogram(delta_max,'BinMethod','auto');
title('\delta_{lat}');

figure
histogram(over,'BinMethod','auto')
xlim([-3,3]);
ylim([0,12000]);
title('Percentage Overshoot');


figure
histogram(set,'BinMethod','auto')
title('Settling time');




toc





