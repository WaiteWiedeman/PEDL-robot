function xdot = robot_xdot(x, F, fc, sysParams)
    th0 = x(1);
    th0d = x(2);
    th1 = x(3);
    th1d = x(4);
    th2 = x(5);
    th2d = x(6);
    
    % system parameters
    L1 = sysParams.L1;
    L2 = sysParams.L2;
    G = sysParams.G;
    M0 = sysParams.M0;
    M1 = sysParams.M1;
    M2 = sysParams.M2;
    l1 = L1/2;
    l2 = L2/2;
    I1 = (1/12)*M1*L1^2;
    I2 = (1/12)*M2*L2^2;

    % solve the Lagrange equation F = D*thdd + C*thd + G
    % compute qdd: D*thdd = F - C*thd - G, using linsolve
    D = [                M0+M1+M2                     -((M1*l1+M2*L1)*sin(th1)+M2*l2*sin(th1+th2))         -M2*l2*sin(th1+th2)
        -l2*M2*sin(th1+th2)-(L1*M2+l1*M1)*sin(th1)   L1^2*M2+l1^2*M1+l2^2*M2+2*L1*l2*M2*cos(th2)+I1       l2^2*M2+L1*l2*M2*cos(th2)
                   -l2*M2*sin(th1+th2)                           l2^2*M2+L1*l2*M2*cos(th2)                      M2*l2^2 + I2       ];
    C = [        0             -((l1*M1+L1*M2)*cos(th1)+M2*l2*cos(th1+th2))*th1d   -M2*l2*cos(th1+th2)*(2*th1d+th2d)
                 0                                    0                             -L1*l2*M2*sin(th2)*(2*th1d+th2d)
                 0                           L1*l2*M2*sin(th2)*th1d                                  0               ];
    G = [                      0
         (L1*M2+l1*M1)*G*cos(th1)+G*l2*M2*cos(th1+th2)
                      G*l2*M2*cos(th1+th2)            ];
    thd = [th0d 
           th1d
           th2d];
    
    F = F - [fc; 0; 0];

    B = F - C*thd - G;
    thdd = linsolve(D,B);

    xdot = zeros(6,1);
    xdot(1) = th0d;
    xdot(2) = thdd(1);
    xdot(3) = th1d;
    xdot(4) = thdd(2);
    xdot(5) = th2d;
    xdot(6) = thdd(3);
end