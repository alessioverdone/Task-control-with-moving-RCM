function [tau] = NewEul(N,L,m,I,fv,fc,g,q,qd,qdd)

    [tau,~,~,~,~,~,~,~,~] = NewEul_Aux(N,L,m,I,fv,fc,g,q,qd,qdd);