function y = run_simscape()
out = sim("robot_model.slx");
t = out.tout;
numTime = length(t);
y = zeros(numTime, 18);
y(:,1) = out.tout; % t
y(:,2) = out.yout{1}.Values.Data; % th0
y(:,3) = out.yout{2}.Values.Data; % th1
y(:,4) = out.yout{3}.Values.Data; % th2
y(:,5) = out.yout{4}.Values.Data; % th0dot
y(:,6) = out.yout{5}.Values.Data; % th1dot
y(:,7) = out.yout{6}.Values.Data; % th2dot
y(:,8) = out.yout{7}.Values.Data; % th0ddot
y(:,9) = out.yout{8}.Values.Data; % th1ddot
y(:,10) = out.yout{9}.Values.Data; % th2ddot
y(:,11) = out.yout{10}.Values.Data; % u
y(:,12) = out.yout{11}.Values.Data; % t1
y(:,13) = out.yout{12}.Values.Data; % t2
y(:,14) = out.yout{13}.Values.Data; % Coulomb Friction
y(:,15) = out.yout{14}.Values.Data; % X desired
y(:,16) = out.yout{15}.Values.Data; % Y desired
y(:,17) = out.yout{16}.Values.Data; % Th1 desired
y(:,18) = out.yout{17}.Values.Data; % Th2 desired 