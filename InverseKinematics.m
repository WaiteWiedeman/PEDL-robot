function [Th1,Th2,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xc,Xcd)
sysParams = params_system();

l1 = sysParams.L1;
l2 = sysParams.L2;

Th2 = acos(((Xd - Xc)^2 + Yd^2 - l1^2 - l2^2)/(2*l1*l2)); %Xd^2 + 

s_Theta2 = sin(Th2);
c_Theta2 = cos(Th2);

Th1 = atan2(Yd,(Xd - Xc)) - atan2(l2*s_Theta2,(l1+l2*c_Theta2)); %Xd

Jac = [ -sin(Th1)*(l1 + l2*cos(Th2)) - cos(Th1)*sin(Th2)*l2  -cos(Th1)*sin(Th2)*l2 - sin(Th1)*cos(Th2)*l2
         cos(Th1)*(l1 + l2*cos(Th2)) - sin(Th1)*sin(Th2)*l2  -sin(Th1)*sin(Th2)*l2 + cos(Th1)*cos(Th2)*l2];
Oms = pinv(Jac)*[Xdd - Xcd ; Ydd];

Om1 = Oms(1);
Om2 = Oms(2);
