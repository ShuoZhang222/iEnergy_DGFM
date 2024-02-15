%% basic system parameters
clear;clc;
Vg_rms=110; % RMS value of the phase voltage
Vg=Vg_rms*sqrt(2);

v_dc_ref=380;

freq=60; % 60 Hz
omega_ref = freq*2*pi;
f_s= 10e3;


T_d= 0.75/f_s; % total delay induced by modulation


%%  passive component values of the VSC power stage
L_f=10e-3;
R_f=0.1;
C_f=200e-6;

L_g=10e-3;
R_g=1; 

C_d=1.5e-3; % DC-link capacitance

Pdc_nom= 5e3; % VSC power rating is 5 kW


%% Design PI parameters for current control
syms K_pi K_ii s1 w1


PMi=65;

w1=2*pi*f_s/10;  % 1 kHz bandwidth for inner current controller
s1=w1*1j;

%TF_delay= (T_d^2/12*s1^2 -T_d/2*s1 +1)/(T_d^2/12*s1^2 +T_d/2*s1 +1);
TF_delay= (-T_d/4*s1+1)/(T_d/4*s1+1); 

Gi_vd=(K_pi+K_ii/s1)*(1/(s1*L_f+R_f))*TF_delay;

ab=abs(Gi_vd);
an=angle(Gi_vd);

exp1=[ab==1,an==-(180-PMi)/(180/pi)];
solu1=vpasolve(exp1,[K_pi,K_ii]);

K_pi=double(solu1.K_pi)
K_ii=double(solu1.K_ii)

%% Current control loop test
% T_d= 0.75/f_s;
G_PI=tf([K_pi K_ii],[1 0]);
G_sys_i= tf([1],[L_f R_f]);
G_delay= tf( [-T_d/4,1], [T_d/4,1]);

figure(1);
bode(G_PI*G_sys_i*G_delay,{10,30e3}); % open loop tf bode
figure(2);
step(G_PI*G_sys_i*G_delay/(1+G_PI*G_sys_i*G_delay)); % close loop step response


%% Design PI parameters for PCC voltage control

syms K_pv K_iv s2 w2

PMv=70;

w2=2*pi*f_s/10/10; % 100 Hz bandwidth for inner voltage controller
s2=w2*1j;

Gv=(K_pv+K_iv/s2)*(1/(s2*C_f)); 



ab_v=abs(Gv);
an_v=angle(Gv);

exp2=[ab_v==1,an_v==-(180-PMv)/(180/pi)];
solu2=vpasolve(exp2,[K_pv,K_iv]);

K_pv=double(solu2.K_pv)
K_iv=double(solu2.K_iv)


%% Voltage control loop test
Gi_cl=G_PI*G_sys_i*G_delay/(1+G_PI*G_sys_i*G_delay);
Gv_PI=tf([K_pv K_iv],[1 0]);
G_sys_v= tf([1],[C_f 0]);

Gv_op=Gv_PI*G_sys_v* Gi_cl;
Gv_cl=Gv_op/(1+Gv_op);

figure(3);
bode(Gv_op);
figure(4);
step(Gv_cl);


%% Design LC DVSC parameters
Kp= 0.01*omega_ref/(0.05*v_dc_ref)
Rdc=0.2;
Rv=v_dc_ref^2*0.05/(Pdc_nom/2)-Rdc

syms omega_c Kd; 

PM_5= 85; % 85 degree phase margin.
w5=  2*pi*2; % 2 Hz cut-off freq.
s5=w5*1j;

Pmax=3/2*Vg^2/(omega_ref*L_g);
G_op5= Pmax*omega_c*(Rv+Rdc)/(v_dc_ref)*(Kp+s5*Kd)/(s5*(s5+omega_c));

ab_5=abs(G_op5);
an_5=angle(G_op5);

exp5=[ab_5==1,an_5==-(180-PM_5)/(180/pi)];
solu5=vpasolve(exp5,[omega_c,Kd]);

omega_c= double(solu5.omega_c)
Kd=     double(solu5.Kd)


%% test the open-loop transfer function of the LC DVSC 
G_op4_1= Pmax*tf(1,[1 0])*(Rv+Rdc)/v_dc_ref;

G_op4_2= omega_c*  tf([Kd,Kp],[1,omega_c]);

G_op4= G_op4_1* G_op4_2;

T_op4 = feedback(G_op4,1);

figure(107); subplot(122);
step(T_op4), title('step response');
subplot(121);
bode(G_op4);grid on;



