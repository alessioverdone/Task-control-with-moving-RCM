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
q0= [0 0 0 0 0 0 0 0.1]; 
q1=q0(1);
q2=q0(2);
q3=q0(3);
q4=q0(4);
q5=q0(5);
q6=q0(6);
q7=q0(7);

lambda=q0(8); 

%Gain
K_rcm = 10*eye(3);
K_t = 10*eye(3);
zero = zeros(3,3);
K= [K_t zero; zero K_rcm];

%Kinematic parameters
l0 = 0.0;
l1 = 0.326;
l2 = 0.2;
l3 = 0.2;
l4 = 0.2;
l5 = 0.19;
off = 0.078;%0.078
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
[t,qv]=ode45('RCM',TSPAN,q0a, options);

%Import robot model
kuka = importrobot('iiwa14.urdf');
kuka.DataFormat = 'column';
axes = show(kuka);
axes.CameraPositionMode = 'manual';
axis([-1.5 1.5 -1.5 1.5 -0.5 2 ]);
p2_out = [];
p3_out = [];

%Animation
for s=1:size(t,1)
    qq = qv(s,1:7);
    %%%END EFFECTOR PART INITIAL
    ee = robotics.RigidBody('ee');
    jntf = robotics.Joint('joint');
    setFixedTransform(jntf,[0 0 0 0],'dh');
    ee.Joint = jntf;
    kuka.addBody(ee,'iiwa_link_ee_kuka');
    T3 = kuka.getTransform(qq','ee');
    p3 = T3(1:3,4);
    p3_out = [p3_out; p3'];
    kuka.removeBody('ee');
    %%%END EFFECTOR PART FINAL
    ee = robotics.RigidBody('ee');
    jntf = robotics.Joint('joint');
    setFixedTransform(jntf,[0 0 0.23 0],'dh');
    ee.Joint = jntf;
    kuka.addBody(ee,'iiwa_link_ee_kuka');
    T3 = kuka.getTransform(qq','ee');
    p2 = T3(1:3,4);
    p2_out = [p2_out; p2'];
    kuka.removeBody('ee');
    disp(s/size(t,1));
end
show(kuka,qv(1,1:7)','PreservePlot', false);
hold on
for r=1:size(t,1)
    qq = qv(r,1:7);
    pf2 = p2_out(r,:);
    pf3 = p3_out(r,:);
    show(kuka,qq','PreservePlot', false,'Frames','off');
    %Plot RCM point
    plot3(-Xtr,-Ytr,Ztr,'o');
    %Plot end-effector tool
    h1 = plot3([pf3(1),pf2(1)],[pf3(2),pf2(2)],[pf3(3),pf2(3)],'r');
    pause(0.005);
    if r < size(t,1)
        set(h1,'Visible','off');
    end
    s1=sin(qv(r,1));
    s2=sin(qv(r,2));   
    s3=sin(qv(r,3));   
    s4=sin(qv(r,4));   
    s5=sin(qv(r,5));   
    s6=sin(qv(r,6));
    s7=sin(qv(r,7));
    c1=cos(qv(r,1));
    c2=cos(qv(r,2));  
    c3=cos(qv(r,3));  
    c4=cos(qv(r,4));  
    c5=cos(qv(r,5));  
    c6=cos(qv(r,6)); 
    c7=cos(qv(r,7));

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

    x_rcm(r)=p7(1)+qv(r,8)*(p6(1)-p7(1));
    y_rcm(r)=p7(2)+qv(r,8)*(p6(2)-p7(2));
    z_rcm(r)=p7(3)+qv(r,8)*(p6(3)-p7(3));
    
    err(r)=sqrt(x_rcm(r)^2+y_rcm(r)^2+z_rcm(r)^2);
    errx_Rcm(r) = Xtr - x_rcm(r);
    erry_Rcm(r) = Ytr - y_rcm(r);
    errz_Rcm(r) = Ztr - z_rcm(r);
    err8(r)=sqrt(errx_Rcm(r)^2+erry_Rcm(r)^2+errz_Rcm(r)^2);
    disp(err8(r));
end
hold off;

%Plot the RCM task error (just as check)
figure(2);
plot(t,errx_Rcm,'.')
hold
figure(3)
plot(t,erry_Rcm,'o')
hold
figure(4)
plot(t,errz_Rcm,'O')
hold
