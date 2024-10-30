function [Xd, Yd, Xdd, Ydd, Xc, Xcd] = referenceTrajectory(t,ctrlParams)
    % rad = ctrlParams.refrad;
    Xd = ctrlParams.refx; % + rad*sin((2*pi/5)*t + pi/2);
    Yd = ctrlParams.refy; % + rad*cos((2*pi/5)*t + pi/2);
    Xdd = 0; %-(2*pi/5)*rad*sin((2*pi/5)*t);
    Ydd = 0; %-(2*pi/5)*rad*cos((2*pi/5)*t);
    
    XcLim = ctrlParams.a - 2;

    if Xd > XcLim
        Xc = XcLim;
    elseif Xd < -XcLim
        Xc = -XcLim;
    else
        Xc = Xd;
    end

    Xcd = Xdd;
end
