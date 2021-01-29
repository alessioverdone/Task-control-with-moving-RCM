function [tau,W,Wd,Wmd,Pdd,Pcdd,Rici,Rij,Rot] = NewEul_Aux(N,L,m,I,fv,fc,g,q,qd,qdd)


%% Gear reduction ratio
kri = 100;
%% Rotor inertia
Im = 0.01;
%% Rotor mass
mm = 0.5;%0.5;

[W,Wd,Wmd,Pdd,Pcdd,Rici,Rij,Rot] = NewEulForw(N,L,g,kri,q,qd,qdd);


tau = NewEulBack(N,m,I,mm,Im,kri,q,qd,qdd,fv,fc,W,Wd,Wmd,Pcdd,Rij,Rici,Rot);
    
    
    
    
    