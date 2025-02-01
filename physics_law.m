function F = physics_law(Y,sysParams)
    sz = size(Y);
    F = zeros(sz(2),3);

    th0 = Y(1,:); % cart position
    th1 = Y(2,:); % link 1 position
    th2 = Y(3,:); % link 2 position
    th0d = Y(4,:); % cart velocity
    th1d = Y(5,:); % link 1 velocity
    th2d = Y(6,:); % link 2 velocity
    th0dd = Y(7,:); % cart acceleration
    th1dd = Y(8,:); % link 1 acceleration
    th2dd = Y(9,:); % link 2 acceleration
    
    % state vectors
    % thd = [th0d; th1d; th2d];
    % thdd = [th0dd; th1dd; th2dd];

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

    % columb friction
    ctrlParams = params_control();
    fc = coulomb_friction(th0d, sysParams, ctrlParams.friction);

    % solve the Lagrange equation F = D*thdd + C*thd + G
    % D = [                M0+M1+M2                     -((M1*l1+M2*L1)*sin(th1)+M2*l2*sin(th1+th2))         -M2*l2*sin(th1+th2)
    %     -l2*M2*sin(th1+th2)-(L1*M2+l1*M1)*sin(th1)   L1^2*M2+l1^2*M1+l2^2*M2+2*L1*l2*M2*cos(th2)+I1       l2^2*M2+L1*l2*M2*cos(th2)
    %                -l2*M2*sin(th1+th2)                           l2^2*M2+L1*l2*M2*cos(th2)                      M2*l2^2 + I2       ];
    % C = [        0             -((l1*M1+L1*M2)*cos(th1)+M2*l2*cos(th1+th2))*th1d   -M2*l2*cos(th1+th2)*(2*th1d+th2d)
    %              0                                    0                             -L1*l2*M2*sin(th2)*(2*th1d+th2d)
    %              0                           L1*l2*M2*sin(th2)*th1d                                  0               ];
    % G = [                      0
    %      (L1*M2+l1*M1)*G*cos(th1)+G*l2*M2*cos(th1+th2)
    %                   G*l2*M2*cos(th1+th2)            ];
    % 
    % F = D*thdd + C*thd + G + [fc;0;0];
    F(:,1) = (M0+M1+M2).*th0dd + (-((M1*l1+M2*L1)*sin(th1)+M2*l2*sin(th1+th2))).*th1dd + -M2*l2*sin(th1+th2).*th2dd ...
        + -((l1*M1+L1*M2)*cos(th1)+M2*l2*cos(th1+th2)).*th1d.^2 + -M2*l2*cos(th1+th2).*(2*th1d+th2d).*th2d + fc;
    F(:,2) = (-l2*M2*sin(th1+th2)-(L1*M2+l1*M1)*sin(th1)).*th0dd + (L1^2*M2+l1^2*M1+l2^2*M2+2*L1*l2*M2*cos(th2)+I1).*th1dd + (l2^2*M2+L1*l2*M2*cos(th2)).*th2dd ...
        + (-L1*l2*M2*sin(th2).*(2*th1d+th2d)).*th2d + (L1*M2+l1*M1)*G*cos(th1)+G*l2*M2*cos(th1+th2);
    F(:,3) = -l2*M2*sin(th1+th2).*th0dd + (l2^2*M2+L1*l2*M2*cos(th2)).*th1dd + (M2*l2^2 + I2).*th2dd ...
        + L1*l2*M2*sin(th2).*th1d.^2 + G*l2*M2*cos(th1+th2);  
end