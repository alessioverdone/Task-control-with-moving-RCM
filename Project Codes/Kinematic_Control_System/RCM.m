function q=KUKAdynwRCM(t,q)

%MV: HERE THE EQUATIONS OF THE MOTION GENERATION SCHEME ARE INTEGRATED
global Xtr Ytr Ztr K l0 l1 l2 l3 l4 l5 off l7 target_pos 

%Robot configuration
q1=q(1)
q2=q(2)
q3=q(3)
q4=q(4)
q5=q(5)
q6=q(6)
q7=q(7)
lambda=q(8)

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


linJ7 = J_7(1:3,:);


% RCM kinematics
J_lambda=(1-lambda)*linJ7+lambda*linJ6;
Jt = zeros(3,8);
Jt(:,1:7)= linJ6
g=p6-p7;

J_RCM=[J_lambda, g];
J_a = [J_RCM; Jt];


% RCM and additional task errors
x_rcm=p7(1)+lambda*(p6(1)-p7(1));
y_rcm=p7(2)+lambda*(p6(2)-p7(2));
z_rcm=p7(3)+lambda*(p6(3)-p7(3));

err_lam=[target_pos(1)-x_rcm;target_pos(2)-y_rcm;target_pos(3)-z_rcm]; 
err_t = [Xtr - p6(1); Ytr - p6(2); Ztr + l7 - p6(3)];
err = [err_lam; err_t];


%Pose and orientation control with RCM constraint
I=eye(8,8);
Jp=pinv(J_a);
w=-10*[0;0;0;0;0;0;0;lambda-0.2];
U=Jp*K*[err]+(I-Jp*J_a)*w;%*[0;0;0;0;0;0,lambda-0.5]*(-10);

q=[U(1);U(2);U(3);U(4);U(5);U(6);U(7); U(8)];



