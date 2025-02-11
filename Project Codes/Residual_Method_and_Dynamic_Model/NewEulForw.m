function [W,Wd,Wmd,Pdd,Pcdd,Rici,Rij,Rot] = NewEulForw(N,L,g,kri,q,qd,qdd)
    
ee_lenght=1;    
[r1,r2,r3,r4,r5,r6,r7,a1,a2,a3,a4,a5,a6,a7,T] = kinematicKuka(q,L,ee_lenght);


%% Out for backwards phase
W = {};%angular velocity 
Wd = {};%angular acceleration
Wmd = {};%rotor angular acceleration
Pdd = {};%linear acceleration
Pcdd = {};%linear com acceleratiom

Rici = {};
Rij = {};
Rot = {r1 r2 r3 r4 r5 r6 r7};
A = {a1 a2 a3 a4 a5 a6 a7};

%% Initialisation and temp vars
prev_w = zeros(3,1);
prev_wd = zeros(3,1);
prev_pdd = zeros(3,1) - g;

curr_w = zeros(3,1);
curr_wd = zeros(3,1);
curr_pdd = zeros(3,1);

curr_pcdd = zeros(3,1);
curr_wmd = zeros(3,1);



%% Forward phase
z0 = [0;0;1];
for i=(1:N)%,
    
    %% Vector from origin of frame i to the center of mass of link i
    %% (Negative quantity, Siciliano's book)
    rici = [-L(i)/2;0;0];
    
    %% Vector from origin of frame i-1 (frame of link i) to origin of
    %% frame j (aka i, meaning that j = i+1 = i) - Siciliano's book
    a = A{i};
    rij = a(1:3,4);
    
    % R from frame i to frame i-1 (1->0,2->1,...). I`m going to use it`s inverse 
    R = Rot{i};
    
    %% Angular velocity of link i with respect to frame i. Equation 7.107 (Siciliano)
    curr_w = R'*(prev_w + qd(i)*z0);%ang.vel. of link i expressed in frame i
    
    %% Angular acceleration with respect to frame i. Equation 7.108 (Siciliano)
    curr_wd = R'*(prev_wd + qdd(i)*z0 + qd(i)*cross(prev_w,z0));%ang.acc. of link i exp. in frame i
    
    %% Linear acceleration with respect to frame i. Equation 7.109 (Siciliano)
    curr_pdd = R'*prev_pdd + cross(curr_wd,rij) + cross(curr_w,cross(curr_w,rij));%%%%%%%%%%%%%%rij???
    %lin.acc. of link i exp. in frame i
    %% Linear acceleration of COM with respect to frame i. Equation 7.110 (Siciliano)
    curr_pcdd = curr_pdd + cross(curr_wd,rici) + cross(curr_w,cross(curr_w,rici));
    
    %% Angular acceleration of the rotor i with respect to frame i-1. Equation 7.111 (Siciliano)
    curr_wmd = prev_wd + kri*qdd(i)*z0 + kri*qd(i)*cross(prev_w,z0);%%%%curr_wmd
    
    %List of velocities and accelerations to pass to bw step
    Rot{i} = R;
    
    W{i} = curr_w;
    Wd{i} = curr_wd;
    Pdd{i} = curr_pdd;
    Pcdd{i} = curr_pcdd;
    Wmd{i} = curr_wmd;
    Rij{i} = rij;
    Rici{i} = rici;
    prev_w = curr_w;
    prev_wd = curr_wd;
    prev_pdd = curr_pdd;
    
   
end

%% Same assumption as Siciliano's book R(N+1->N) = Identity
Rot = [Rot eye(3,3)];