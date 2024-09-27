function [x1,y1,x2,y2,xend,yend] = ForwardKinematics(x,sysParams)
    L1 = sysParams.L1;
    L2 = sysParams.L2;
    l1 = L1/2;
    l2 = L2/2;

    th0 = x(:,1);
    th1 = x(:,2);
    th2 = x(:,3);

    x1 = th0 + l1*cos(th1);
    y1 = l1*sin(th1);
    x2 = th0 + L1*cos(th1) + l2*cos(th1+th2);
    y2 = L1*sin(th1) + l2*sin(th1+th2);
    xend = th0 + L1*cos(th1) + L2*cos(th1+th2);
    yend = L1*sin(th1) + L2*sin(th1+th2);
end
