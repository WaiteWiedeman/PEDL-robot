function dxdt = robot_system(t, x, sysParams, ctrlParams)
    [Xd, Yd, Xdd, Ydd] = referenceTrajectory(t, ctrlParams);
    F = force_function(t, x, Xd, Yd, Xdd, Ydd,ctrlParams);
    dxdt = robot_xdot(x, F, sysParams);
end