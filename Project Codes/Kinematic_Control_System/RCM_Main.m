%MV: THIS IS THE MAIN FILE

clear all
close all
clc

%Global variables
global Xtr Ytr Ztr K l0 l1 l2 l3 l4 l5 off l7  target_pos

%Trocar position
Xtr=0.5;
Ytr=0.5;
Ztr=0.0;

%Initial robot configuration q=[q1 q2 q3 q4 q5 q6 q7 lambda]
q0= [0 0 0 0 0 0 0 0.0]; 
q1=q0(1)
q2=q0(2)
q3=q0(3)
q4=q0(4)
q5=q0(5)
q6=q0(6)
q7=q0(7)

lambda=q0(8) 

%Gain
K_ = 1;
K_rcm = K_*eye(3);
K_t = K_*eye(3);
zero = zeros(3,3);
K= [K_t zero; zero K_rcm];

%Kinematic parameters
l0 = 0.0;
l1 = 0.326;
l2 = 0.2;
l3 = 0.2;
l4 = 0.2;
l5 = 0.19;
off = 0.078;
l7 = 0.23;

%Target position
target_pos=[Xtr;Ytr;Ztr];

%Joint positions (used for animation)
s1=sin(q1);
s2=sin(q2);   
s3=sin(q3);   
s4=sin(q4);   
s5=sin(q5);   
s6=sin(q6);
s7=sin(q7);
c1=cos(q1);
c2=cos(q2);  
c3=cos(q3);  
c4=cos(q4);  
c5=cos(q5);  
c6=cos(q6); 
c7=cos(q7);

p0=[0;0;0];

p1 = [0; 0; l0+l1];

p2 = [0; 0; l0+l1];

p3 = [-c1*c2*(l2+l3); -s1*s2*(l2+l3); c2*(l2+l3)+(l0+l1)*(l2+l3)];

p4 = [-c1*c2*(l2+l3); -s1*s2*(l2+l3); c2*(l2+l3)+(l0+l1)*(l2+l3)];

p5 = [(c1*c2*c3*s4 - s1*s3*s4 - c1*s2*c4)*(l4+l5) - c1*s2*(l2+l3);...
      (s1*c2*c3*s4 + c1*s3*s4 - s1*s2*c4)*(l4+l5) - s1*s2*(l2+l3);...
      (s2*c3*s4 + c2*c4)*(l4*l5) + c2*(l2+l3) + (l0+l1)*(l2+l3)];
  
p6 = [(c1*c2*c3*s4 - s1*s3*s4 - c1*s2*c4)*(l4+l5) - c1*s2*(l2+l3);...
      (s1*c2*c3*s4 + c1*s3*s4 - s1*s2*c4)*(l4+l5) - s1*s2*(l2+l3);...
      (s2*c3*s4 + c2*c4)*(l4*l5) + c2*(l2+l3) + (l0+l1)*(l2+l3)];
  
p7 = [(-c1*c2*c3*c4*c5*s6 + s1*s3*c4*c5*s6 - c1*s2*s4*c5*s6 + c1*c2*s3*s5*s6 + s1*c3*s5*s6 + c1*c2*c3*s4*c6 - s1*s3*s4*c6 - c1*s2*s4*c6)*(l7 + off) + p6(1);...
      (-s1*c2*c3*c4*c5*s6 - c1*s3*c4*c5*s6 - s1*s2*s4*c5*s6 + s1*c2*s3*s5*s6 - c1*c3*s5*s6 + s1*c2*c3*s4*c6 + c1*s3*s4*c6 - s1*s2*c4*c6)*(l7 + off) + p6(2);...
      (-s2*c3*c4*c5*s6 + c2*s4*c5*s6 + s2*s3*s5*s6 + s2*c3*s4*c6 + c2*c4*c6)*(l7 + off) + p6(3)];


%Control Integration
q0a=[q0];

TSPAN=[0 5];
tol=1e-6;
options=odeset('RelTol',tol,'AbsTol',[tol tol tol tol tol tol tol tol]);
[t,q]=ode45('RCM',TSPAN,q0a, options);

%Save data
f1d=fopen('err_RCM.txt','wt');
f2d=fopen('err_ADD_TASK.txt','wt');
f3d=fopen('q_dot.txt','wt');
f4d=fopen('pd_rcm.txt','wt');
f7d=fopen('p_rcm.txt','wt');
f8d=fopen('q.txt','wt');


% for i=1:size(t,1)
%     
%     if i == 1
%         qd_1(i) = 0;
%     end
%     
%     qd_1(i) = (q(i,1) - 0)/(t(i) - 0);
%     
%     
%     fprintf(f9d,'%f %f\n',[t(i) qd_1(i)]);
% end 


%Animation
for i=1:size(t,1)
    
    disp(q(i,:));
    
    s1=sin(q(i,1));
    s2=sin(q(i,2));   
    s3=sin(q(i,3));   
    s4=sin(q(i,4));   
    s5=sin(q(i,5));   
    s6=sin(q(i,6));
    s7=sin(q(i,7));
    c1=cos(q(i,1));
    c2=cos(q(i,2));  
    c3=cos(q(i,3));  
    c4=cos(q(i,4));  
    c5=cos(q(i,5));  
    c6=cos(q(i,6)); 
    c7=cos(q(i,7));

    %Joints positions

    p0 = [0; 0; 0];

    p1 = [0; 0; l0+l1];

    p2 = [0; 0; l0+l1];

    p3 = [-c1*c2*(l2+l3); -s1*s2*(l2+l3); c2*(l2+l3)+(l0+l1)*(l2+l3)];

    p4 = [-c1*c2*(l2+l3); -s1*s2*(l2+l3); c2*(l2+l3)+(l0+l1)*(l2+l3)];

    p5 = [(c1*c2*c3*s4 - s1*s3*s4 - c1*s2*c4)*(l4+l5) - c1*s2*(l2+l3);...
          (s1*c2*c3*s4 + c1*s3*s4 - s1*s2*c4)*(l4+l5) - s1*s2*(l2+l3);...
          (s2*c3*s4 + c2*c4)*(l4*l5) + c2*(l2+l3) + (l0+l1)*(l2+l3)];

    p6 = [(c1*c2*c3*s4 - s1*s3*s4 - c1*s2*c4)*(l4+l5) - c1*s2*(l2+l3);...
          (s1*c2*c3*s4 + c1*s3*s4 - s1*s2*c4)*(l4+l5) - s1*s2*(l2+l3);...
          (s2*c3*s4 + c2*c4)*(l4*l5) + c2*(l2+l3) + (l0+l1)*(l2+l3)];

    p7 = [(-c1*c2*c3*c4*c5*s6 + s1*s3*c4*c5*s6 - c1*s2*s4*c5*s6 + c1*c2*s3*s5*s6 + s1*c3*s5*s6 + c1*c2*c3*s4*c6 - s1*s3*s4*c6 - c1*s2*s4*c6)*(l7 + off) + p6(1);...
          (-s1*c2*c3*c4*c5*s6 - c1*s3*c4*c5*s6 - s1*s2*s4*c5*s6 + s1*c2*s3*s5*s6 - c1*c3*s5*s6 + s1*c2*c3*s4*c6 + c1*s3*s4*c6 - s1*s2*c4*c6)*(l7 + off) + p6(2);...
          (-s2*c3*c4*c5*s6 + c2*s4*c5*s6 + s2*s3*s5*s6 + s2*c3*s4*c6 + c2*c4*c6)*(l7 + off) + p6(3)];
    
    %Jacobians

    J_6 = zeros(6,7);

    J_6(1,1)=-(l4 + l5)*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + (l2 + l3)*s1*s2;
    J_6(2,1)=-(l4 + l5)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (l2 + l3)*c1*s2;
    J_6(6,1)=1;

    J_6(1,2)=-c1*((l4 + l5)*(c2*c4 + c3*s2*s4) + (l2 + l3)*c2);
    J_6(2,2)=-s1*((l4 + l5)*(c2*c4 + c3*s2*s4) + (l2 + l3)*c2);
    J_6(3,2)=-(l2 + l3)*s2 - (l4 + l5)*c4*s2 + (l4 + l5)*c2*c3*s4;
    J_6(4,2)=s1;
    J_6(5,2)=-c1;


    J_6(1,3)=-(l4 + l5)*s4*(c3*s1 + c1*c2*s3);
    J_6(2,3)=(l4 + l5)*s4*(c1*c3 - c2*s1*s3);
    J_6(3,3)=-(l4 + l5)*s2*s3*s4;
    J_6(4,3)=-c1*s2;
    J_6(5,3)=-s1*s2;
    J_6(6,3)=c2;

    J_6(1,4)=(l4 + l5)*c1*s2*s4 - (l4 + l5)*c4*s1*s3 + (l4 + l5)*c1*c2*c3*c4;
    J_6(2,4)=(l4 + l5)*s1*s2*s4 + (l4 + l5)*c1*c4*s3 + (l4 + l5)*c2*c3*c4*s1;
    J_6(3,4)=-(l4 + l5)*c2*s4 + (l4 + l5)*c3*c4*s2;
    J_6(4,4)=-c3*s1 - c1*c2*s3;
    J_6(5,4)=-c2*s1*s3 + c1*c3;
    J_6(6,4)=-s2*s3;

    J_6(4,5)=-s4*(s1*s3 - c1*c2*c3) - c1*c4*s2;
    J_6(5,5)=s4*(c1*s3 + c2*c3*s1) - c4*s1*s2;
    J_6(6,5)=c2*c4 + c3*s2*s4;

    J_6(4,6)=c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4);
    J_6(5,6)=s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3);
    J_6(6,6)=-s5*(c2*s4 - c3*c4*s2) + c5*s2*s3;

    linJ6 = J_6(1:3,:);

    J_7 = zeros(6,7);

    J_7(1,1)=-(l4 + l5)*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - (off + l7)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + (l2 + l3)*s1*s2;
    J_7(2,1)=-(l4 + l5)*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - (off + l7)*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - (l2 + l3)*c1*s2;
    J_7(6,1)=1;

    J_7(1,2)=-(off + l7)*c1*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) - (l2 + l3)*c1*c2 - (l4 + l5)*c1*(c2*c4 + c3*s2*s4);
    J_7(2,2)=-(off + l7)*s1*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) - (l2 + l3)*c2*s1 - (l4 + l5)*s1*(c2*c4 + c3*s2*s4);
    J_7(3,2)=(off + l7)*c2*s3*s5*s6 - (l4 + l5)*c4*s2 + (l4 + l5)*c2*c3*s4 - (off + l7)*c4*c6*s2 + (off + l7)*c2*c3*c6*s4 - (l2 + l3)*s2 - (off + l7)*c5*s2*s4*s6 - (off + l7)*c2*c3*c4*c5*s6;
    J_7(4,2)=s1;
    J_7(5,2)=-c1;

    J_7(1,3)=-(l4 + l5)*c3*s1*s4 - (l4 + l5)*c1*c2*s3*s4 + (off + l7)*c3*c6*s1*s4 - (off + l7)*s1*s3*s5*s6 - (off + l7)*c1*c2*c6*s3*s4 + (off + l7)*c1*c2*c3*s5*s6 + (off + l7)*c3*c4*c5*s1*s6 + (off + l7)*c1*c2*c4*c5*s3*s6;
    J_7(2,3)=-(l4 + l5)*c2*s1*s3*s4 + (off + l7)*c1*c3*c6*s4 + (l4 + l5)*c1*c3*s4 + (off + l7)*c1*s3*s5*s6 - (off + l7)*c1*c3*c4*c5*s6 - (off + l7)*c2*c6*s1*s3*s4 + (off + l7)*c2*c3*s1*s5*s6 + (off + l7)*c2*c4*c5*s1*s3*s6;
    J_7(3,3)=-s2*((l4 + l5)*s3*s4 + (off + l7)*c6*s3*s4 - (off + l7)*c3*s5*s6 - (off + l7)*c4*c5*s3*s6);
    J_7(4,3)=-c1*s2;
    J_7(5,3)=-s1*s2;
    J_7(6,3)=c2;

    J_7(1,4)=(l4 + l5)*c1*s2*s4 - (l4 + l5)*c4*s1*s3 + (l4 + l5)*c1*c2*c3*c4 + (off + l7)*c1*c6*s2*s4 - (off + l7)*c4*c6*s1*s3 - (off + l7)*c1*c4*c5*s2*s6 - (off + l7)*c5*s1*s3*s4*s6 + (off + l7)*c1*c2*c3*c4*c6 + (off + l7)*c1*c2*c3*c5*s4*s6;
    J_7(2,4)=(l4 + l5)*s1*s2*s4 + (l4 + l5)*c1*c4*s3 + (l4 + l5)*c2*c3*c4*s1 + (off + l7)*c1*c4*c6*s3 + (off + l7)*c6*s1*s2*s4 + (off + l7)*c2*c3*c4*c6*s1 - (off + l7)*c4*c5*s1*s2*s6 + (off + l7)*c1*c5*s3*s4*s6 + (off + l7)*c2*c3*c5*s1*s4*s6;
    J_7(3,4)=(off + l7)*c3*c5*s2*s4*s6 + (l4 + l5)*c3*c4*s2 - (off + l7)*c2*c6*s4 + (off + l7)*c3*c4*c6*s2 + (off + l7)*c2*c4*c5*s6 - (l4 + l5)*c2*s4;
    J_7(4,4)=-c3*s1 - c1*c2*s3;
    J_7(5,4)=-c2*s1*s3 + c1*c3;
    J_7(6,4)=-s2*s3;

    J_7(1,5)=(off + l7)*s6*(c3*c5*s1 + c1*c2*c5*s3 + c1*s2*s4*s5 - c4*s1*s3*s5 + c1*c2*c3*c4*s5);
    J_7(2,5)=(off + l7)*s6*(c2*c5*s1*s3 - c1*c3*c5 + c1*c4*s3*s5 + s1*s2*s4*s5 + c2*c3*c4*s1*s5);
    J_7(3,5)=(off + l7)*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5);
    J_7(4,5)=-s4*(s1*s3 - c1*c2*c3) - c1*c4*s2;
    J_7(5,5)=s4*(c1*s3 + c2*c3*s1) - c4*s1*s2;
    J_7(6,5)=c2*c4 + c3*s2*s4;

    J_7(1,6)=(off + l7)*c1*c4*s2*s6 + (off + l7)*c3*c6*s1*s5 + (off + l7)*s1*s3*s4*s6 - (off + l7)*c1*c2*c3*s4*s6 + (off + l7)*c1*c2*c6*s3*s5 - (off + l7)*c1*c5*c6*s2*s4 + (off + l7)*c4*c5*c6*s1*s3 - (off + l7)*c1*c2*c3*c4*c5*c6;
    J_7(2,6)=(off + l7)*c4*s1*s2*s6 - (off + l7)*c1*c3*c6*s5 - (off + l7)*c1*s3*s4*s6 - (off + l7)*c1*c4*c5*c6*s3 - (off + l7)*c2*c3*s1*s4*s6 + (off + l7)*c2*c6*s1*s3*s5 - (off + l7)*c5*c6*s1*s2*s4 - (off + l7)*c2*c3*c4*c5*c6*s1;
    J_7(3,6)=-(off + l7)*c3*s2*s4*s6 + (off + l7)*c2*c5*c6*s4 - (off + l7)*c2*c4*s6 + (off + l7)*c6*s2*s3*s5 - (off + l7)*c3*c4*c5*c6*s2;
    J_7(4,6)=c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4);
    J_7(5,6)=s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3);
    J_7(6,6)=-s5*(c2*s4 - c3*c4*s2) + c5*s2*s3;

    J_7(4,7)=-c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
    J_7(5,7)=c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));
    J_7(6,7)=c6*(c2*c4 + c3*s2*s4) + s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5);

%     if Ztr>p7(3)
%         lambda=0.0;
%     end

    linJ7 = J_7(1:3,:);
    x_rcm(i)=p7(1)+q(i,8)*(p6(1)-p7(1));
    y_rcm(i)=p7(2)+q(i,8)*(p6(2)-p7(2));
    z_rcm(i)=p7(3)+q(i,8)*(p6(3)-p7(3));
    lam(i) = q(i,8);
    
    %err(i)=sqrt(x_rcm(i)^2+y_rcm(i)^2+z_rcm(i)^2);
    errx_Rcm(i) = Xtr - x_rcm(i);
    erry_Rcm(i) = Ytr - y_rcm(i);
    errz_Rcm(i) = Ztr - z_rcm(i);
    
    err_t_x(i) = Xtr - p6(1); 
    err_t_y(i) = Ytr - p6(2);
    err_t_z(i) = Ztr+l7- p6(3);
    
    J_lambda=(1-lam(i))*linJ7+lam(i)*linJ6;
    Jt = zeros(3,8);
    Jt(:,1:7)= linJ6
    g=p6-p7;
    
    err=[errx_Rcm(i); erry_Rcm(i); errz_Rcm(i); err_t_x(i); err_t_y(i); err_t_z(i)]
    J_RCM=[J_lambda, g];
    J_a = [J_RCM; Jt];
    I=eye(8,8);
    Jp=pinv(J_a);
    w=-10*[0;0;0;0;0;0;0;lam(i)-0.2];
    
    if i<size(t,1)-1
        if t(i+1)-t(i)>=0.001
            q_dott(:,i) = Jp*K*[err]+(I-Jp*J_a)*w;
            pd_RCM(:,i) = J_RCM*q_dott(:,i);
        end
    end
 
    
    
    plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],...
    [p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],...
    [p2(1),p3(1)],[p2(2),p3(2)],[p2(3),p3(3)],...
    [p3(1),p4(1)],[p3(2),p4(2)],[p3(3),p4(3)],...
    [p4(1),p5(1)],[p4(2),p5(2)],[p4(3),p5(3)],...
    [p5(1),p6(1)],[p5(2),p6(2)],[p5(3),p6(3)],...
    [p6(1),p7(1)],[p6(2),p7(2)],[p6(3),p7(3)],...
    Xtr,Ytr,Ztr,'o',...
    target_pos(1),target_pos(2),target_pos(3),'O') 
    
    
    cupHeight = 0.05;
    cupRadius = 0.2;
    cupPosition = [Xtr Ytr Ztr]';

    exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
    %axis([-2.0 2.0 -2.0 2.0 -2.0 2.0])
    
pause(0.1)
end

fprintf(f1d,'%f\n',t);
fprintf(f1d,'%c\n','_');
fprintf(f1d,'%f\n',errx_Rcm);
fprintf(f1d,'%c\n','_');
fprintf(f1d,'%f\n',erry_Rcm);
fprintf(f1d,'%c\n','_');
fprintf(f1d,'%f\n',errz_Rcm);

fprintf(f2d,'%f\n',t);
fprintf(f2d,'%c\n','_');
fprintf(f2d,'%f\n',err_t_x);
fprintf(f2d,'%c\n','_');
fprintf(f2d,'%f\n',err_t_y);
fprintf(f2d,'%c\n','_');
fprintf(f2d,'%f\n',err_t_z);

fprintf(f3d,'%f\n',t);
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(1,:));
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(2,:));
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(3,:));
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(4,:));
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(5,:));
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(6,:));
fprintf(f3d,'%c\n','_');
fprintf(f3d,'%f\n',q_dott(7,:));

fprintf(f4d,'%f\n',t);
fprintf(f4d,'%c\n','_');
fprintf(f4d,'%f\n',pd_RCM(1,:));
fprintf(f4d,'%c\n','_');
fprintf(f4d,'%f\n',pd_RCM(2,:));
fprintf(f4d,'%c\n','_');
fprintf(f4d,'%f\n',pd_RCM(3,:));

fclose(f1d);
fclose(f2d);
fclose(f3d);
fclose(f4d);
%fclose(f9d);


%Plot the task error dynamics
% h1 = figure(2)
% plot(t,errx_Rcm,'.')
% title('Error on the RCM task constraint: x-component')
% xlabel('Time')
% ylabel('x-Error')
% savefig(h1, 'Error on the RCM task constraint: x-component.fig');
% hold
% 
% h2 = figure(3)
% plot(t,erry_Rcm,'.')
% title('Error on the RCM task constraint: y-component')
% xlabel('Time')
% ylabel('y-Error')
% savefig(h2, 'Error on the RCM task constraint: y-component.fig');
% hold
% 
% h3 = figure(4)
% plot(t,errz_Rcm,'.')
% title('Error on the RCM task constraint: z-component')
% xlabel('Time')
% ylabel('z-Error')
% savefig(h3, 'Error on the RCM task constraint: z-component.fig');
% hold
% 
h4 = figure(5)
plot(t,err_t_x,'.')
title('Error on the additional task: x-component')
xlabel('Time')
ylabel('x-Error')
%savefig(h4, 'Error on the additional task: x-component.fig');
hold

% h5 = figure(6)
% plot(t,err_t_y,'.')
% title('Error on the additional task: y-component')
% xlabel('Time')
% ylabel('y-Error')
% savefig(h5, 'Error on the additional task: y-component.fig');
% hold
% 
h6 = figure(7)
plot(t,err_t_z,'.')
title('Error on the additional task: z-component')
xlabel('Time')
ylabel('z-Error')
%savefig(h6, 'Error on the additional task: z-component.fig');

hold
%Save RCM position
p_rcm = [x_rcm; y_rcm; z_rcm];
fprintf(f7d,'%f\n',t);
fprintf(f7d,'%c\n','_');
fprintf(f7d,'%f\n',x_rcm);
fprintf(f7d,'%c\n','_');
fprintf(f7d,'%f\n',y_rcm);
fprintf(f7d,'%c\n','_');
fprintf(f7d,'%f\n',z_rcm);
fclose(f7d);

fprintf(f8d,'%f\n',t);
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,1));
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,2));
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,3));
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,4));
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,5));
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,6));
fprintf(f8d,'%c\n','_');
fprintf(f8d,'%f\n',q(:,7));
fprintf(f8d,'%c\n','_');
fclose(f8d);

h7 = figure(8)
plot(t,p_rcm,'.')
hold on
title('RCM')
xlabel('Time')
ylabel('Position')
%savefig(h6, 'Error on the additional task: z-component.fig');
hold

% h7 = figure(8)
% plot(t,lam,'.')
% title('Lambda variation')
% xlabel('Time')
% ylabel('Lambda')
% savefig(h6, 'Lambda_Variation.fig');
% hold

