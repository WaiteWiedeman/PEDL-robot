function params = params_system()
    params = struct();
    params.L1 = 1; % length of link 1
    params.L2 = 1; % length of link 2
    params.G = 9.8; % gravity
    params.M0 = 1;  % point mass of cart
    params.M1 = 7.5; % point mass of link 1
    params.M2 = 7.5; % point mass of link 2
end