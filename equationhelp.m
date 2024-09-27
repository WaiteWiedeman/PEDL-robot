close all; clear; clc;

syms t th0(t) th1(t) th2(t) L1 L2 l1 l2 m0 m1 m2 I1 I2 g

x0 = th0;
y0 = 0;
x1 = th0 + l1*cos(th1);
y1 = l1*sin(th1);
x2 = th0 + L1*cos(th1) + l2*cos(th1+th2);
y2 = L1*sin(th1) + l2*sin(th1+th2);

dth1 = diff(th1,t);
dth2 = diff(th2,t);
dx0 = diff(x0,t);
dx1 = diff(x1,t);
dy1 = diff(y1,t);
dx2 = diff(x2,t);
dy2 = diff(y2,t);

KE0 = (1/2)*m0*dx0^2;
KE1 = (1/2)*m1*(dx1^2+dy1^2) + (1/2)*I1*dth1;
KE2 = (1/2)*m2*(dx2^2+dy2^2) + (1/2)*I2*dth2;

PE0 = 0;
PE1 = m1*g*l1*sin(th1);
PE2 = m2*g*(L1*sin(th1)+l2*sin(th1+th2));

KEtot = KE0 + KE1 + KE2;
PEtot = PE0 + PE1 + PE2;

Lag = KEtot - PEtot;

dLdth0 = diff(Lag,th0);
dLddx0 = diff(Lag,dx0);

dLdth1 = diff(Lag,th1);
dLddth1 = diff(Lag,dth1);

dLdth2 = diff(Lag,th2);
dLddth2 = diff(Lag,dth2);

u = diff(dLddx0,t) - dLdth0;
tau1 = diff(dLddth1,t) - dLdth1;
tau2 = diff(dLddth2,t) - dLdth2;

u = simplify(u);
tau1 = simplify(tau1);
tau2 = simplify(tau2);
