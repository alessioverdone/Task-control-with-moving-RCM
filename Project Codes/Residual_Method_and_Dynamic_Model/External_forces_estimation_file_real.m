function [] = External_forces_estimation_file_real()
clear;
clear all;
clc;
format shortEng;
format compact;
kuka = importrobot('iiwa14.urdf');
kuka.DataFormat = 'column';
%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%
N=7;% # of joints
tau_act = zeros(N,1);
fv = 0.9;
fv_vec = [0.9;0.9;0.9;0.9;0.9;0.9;0.9];
fc = 0.9;
L = [.326 .2 .2 0.2 0.19 0.78 0];%links' lenght
m = [5 4.29968 3.65853 2.38466 1.70355 0.40007 0.65014];%links' mass
% Inertia_param_vector = [Ixx Iyy Izz Ixy Ixz Iyz];
constructI = @(v) [v(1)   v(4)   v(5);
                   v(4)   v(2)   v(6);
                   v(5)   v(6)   v(3)];
      
inertia_param_1 = [0.01 0.000189 0.01 0.01 0.01 0.01];
inertia_param_2 = [0.04741 0.05 0.001601 -0.00000621 0.0001166 -0.000914];
inertia_param_3 = [0.046951 0.0008344 0.05 -0.000577 0.000409 -0.000577];
inertia_param_4 = [0.012423 0.007270 0.009988 -0.000518 0.00000002 -0.000548];
inertia_param_5 = [0.006322 0.001202 0.007080 -0.000216 0.000006 -0.005];
inertia_param_6 = [0.000527 0 0.003489 0.000048 -0.0000375 -0.001030];
inertia_param_7 = [0 0.000032 0.000118 -0.00000005 0 0 ];

I1 = constructI(inertia_param_1);
I2 = constructI(inertia_param_2);
I3 = constructI(inertia_param_3);
I4 = constructI(inertia_param_4);
I5 = constructI(inertia_param_5);
I6 = constructI(inertia_param_6);
I7 = constructI(inertia_param_7);

I = {I1 I2 I3 I4 I5 I6 I7};%inertia matrices
g = [0 -9.81 0]';
%% DeltaTime for Euler integration
deltaTime = 0.25;
tot=50;
time = 0:deltaTime:tot;
q = [ 1.5; 1.5 ; 1.5; 1.5; 1.5; -1.5; 1.5;];
qd = zeros(N,1);
qdd = zeros(N,1);
tau_ext = zeros(N,1);
beta =  zeros(N,1);
q_out = [];
e_q = [];
e_q2 = [];
e_qd = [];
e_qdd = [];
t_tau = [];
t_g_n = [];
list=[];5
p_out=[];
p2_out = [];
p3_out = [];
r = zeros(1,N);
w = 0;
r_t = [];
f_ext_2 = zeros(N,1);
B = [];
K = 1;
r_plot = [];
tau_ext_plot = [];
f_ext_2_tot = [];
error = [];
output = zeros(N,1);
sumDyn = zeros(N,1);
sumRes = zeros(N,1);
f_ext_tot = [];
w = 0;
for t=time
    w = w +1;
    f_ext = [sin(w/30)*50;cos(w/30)*50;0;0;0;0];
    %%Computing Coriolis, Centrifugal and Gravity terms
    q_tmp = q;
    qd_tmp = qd;
    qdd_tmp = zeros(N,1);
    n = NewEul(N,L,m,I,fv,fc,g,q_tmp,qd_tmp,qdd_tmp);
    %%Computing gravity term
    q_tmp = q;%q_d
    qd_tmp = zeros(N,1);
    qdd_tmp = zeros(N,1);
    g_n = NewEul(N,L,m,I,fv,fc,g,q_tmp,qd_tmp,qdd_tmp);
    % Computing inertia matrix B
    B_old = B;
    B = [];
    Binv = [];
    for i=(1:N)
        q_tmp = q;
        qd_tmp = zeros(N,1);
        qdd_tmp = zeros(N,1);
        qdd_tmp(i) = 1;        
        bi = NewEul(N,L,m,I,fv,fc,g,q_tmp,qd_tmp,qdd_tmp);
        B = [B bi];
    end
    B_new = B;
    Binv = inv(B);
    C = NewEul(N,L,m,I,fv,fc,0,q,qd,[0;0;0;0;0;0;0]);
    
   
    
    %%%%%%%%%%%%%%%%%%%%%% RCM PART %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ee = robotics.RigidBody('ee');
    jntf = robotics.Joint('joint');
    setFixedTransform(jntf,[0 0 0.18 0],'dh');%Final point of end-effector tool
    ee.Joint = jntf;
    kuka.addBody(ee,'iiwa_link_ee_kuka');
    T = kuka.getTransform(q,'ee');%This matrix is used to compute Jc
    p = T(1:3,4);
    p_out = [p_out; p'];%used to compute end-effector's tool
    Jc = kuka.geometricJacobian(q,'ee');%Jacobian at the contact point
    tau_ext = Jc' * f_ext;%External torques acting on robot joints
    T2 = kuka.getTransform(q,'iiwa_link_ee_kuka');
    p2 = T2(1:3,4);
    p2_out = [p2_out; p2'];%used to compute end-effector's tool
    kuka.removeBody('ee');
    
     %%%%%%%%%%%%%% CONTROL PART %%%%%%%%%%%%%%%%%%%%%%%%%
    tau = g_n - Jc'*f_ext;% gravity compensation control law
    qdd = Binv * (tau + tau_ext - n);% computing acceleration
    qd = qd + qdd*deltaTime;% computing velocities
    q = q + qd*deltaTime + qdd *(deltaTime^2)*0.5;% computing joint position
    q_out = [q_out; q'];
    if w >1 
        beta = n - ((B_new - B_old)/deltaTime) * qd;%used in the residual method
    end
    %%% END EFFECTOR PART
    ee = robotics.RigidBody('ee');
    jntf = robotics.Joint('joint');
    lambda = 0.28;
    setFixedTransform(jntf,[0 0 lambda 0],'dh');
    ee.Joint = jntf;
    kuka.addBody(ee,'iiwa_link_ee_kuka');

    %%%%%%%%%%%%% RESIDUAL METHOD %%%%%%%%%%%%%%%%%%%%%%%
    sumDyn = sumDyn + (tau - beta)*deltaTime;
    sumRes = sumRes + (output*deltaTime);
    output = K*(B*qd - sumDyn -sumRes - 2*fv_vec )/(1+K*deltaTime);       
    r=output';

    T3 = kuka.getTransform(q,'ee');
    p3 = T3(1:3,4);
    p3_out = [p3_out; p3'];%used to compute end-effector's tool(final point)
    kuka.removeBody('ee');
    %%%%%%%%%%%%%% FORCES ESTIMATION %%%%%%%%%%%%%%%%%%%
    [U,S,V] = svd(Jc');
    %Jdls = 
    f_ext_2 = pinv(Jc')* r';
    if norm(f_ext_2-f_ext) > 50
        disp('Determinante di JC troppo alto');
        disp(det(pinv(Jc*Jc')));
    end
    if w > 4
        %Here we insert the data for final plots only after 4 iterations
        %to avoid scal eproblemsin final plot
        r_plot = [r_plot; r];
        tau_ext_plot = [tau_ext_plot; tau_ext'];
        f_ext_2_tot = [f_ext_2_tot; f_ext_2'];
        error = [ error; norm(f_ext - f_ext_2)];
        f_ext_tot = [f_ext_tot;f_ext'];
        disp(norm(f_ext - f_ext_2));
        
    end
end

%%%%%%%%%%%% SIMULATION PART %%%%%%%%%%%%%%%%%%%%%%%%%55
show(kuka,q_out(1,1:7)','PreservePlot', false);
hold on
for r=1:size(time,2)    
    qf = q_out(r,:);
    pf = p_out(r,:);
    pf2 = p2_out(r,:);
    pf3 = p3_out(r,:);
    show(kuka,qf','PreservePlot', false,'Frames','off');
    %Plot RCM point
    plot3(pf(1),pf(2),pf(3),'.');
    %Plot end-effector tool
    plot3([pf3(1),pf2(1)],[pf3(2),pf2(2)],[pf3(3),pf2(3)],'r');
    pause(0.005)
end
hold off;

%%%%%%%%%% PLOT PART %%%%%%%%%%%%%%%%%%%%%
 fig2 = figure();
 ax1 = subplot(2,1,1);
 plot(ax1,tau_ext_plot);
 title(ax1,'External torque');
 ax2 = subplot(2,1,2);
 plot(ax2,r_plot);
 title(ax2,'Computed residual'); 
 savefig(fig2, "Torque.fig");
 fig3 = figure();
 
 ax3 = subplot(2,1,1);
 plot(ax3,f_ext_tot);
 title(ax3,'External forces'); 
 ax4 = subplot(2,1,2);
 plot(ax4,f_ext_2_tot);
 title(ax4,'Estimated external forces');
 savefig(fig3, "Force.fig");
 fig4 = figure();
 
 ax5 = subplot(1,1,1);
 plot(ax5,error');
 title(ax5,'Forces error');
 savefig(fig4, "Force_err.fig");
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




