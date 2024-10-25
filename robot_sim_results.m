close all; clear; clc;

% Two-link Planar Robot on Cart Dynamics System Parameters
sysParams = params_system();
ctrlParams = params_control();
ctrlParams.method = "origin";
ctrlParams.PID0 = [10000 700 1000]; 
ctrlParams.PID1 = [10000 1500 1000];
ctrlParams.PID2 = [10000 1500 1000];
ctrlParams.refx = 1; % -2 + 4*rand; % center of reference trajectory in x
ctrlParams.refy = 1; % -2 + 4*rand; % center of reference trajectory in y
tSpan = [0,10]; %0:0.01:5;
x0 = [0; 0; 0; 0; 0; 0]; % th0, th0d, th1, th1d, th2, th2d

% run simulation
y = robot_simulation(tSpan, x0, sysParams, ctrlParams);

% run simscape model
% y_simscape = run_simscape();

% plot states, forces, and states against reference
plot_states(y(:,1),y(:,2:10),"position",y(:,15:18));
plot_states(y(:,1),y(:,2:10),"velocity",y(:,15:18));
plot_states(y(:,1),y(:,2:10),"acceleration",y(:,15:18));
plot_forces(y(:,1),y(:,11),y(:,12),y(:,13),y(:,14));

% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:4),sysParams);
plot_endeffector([xend yend],y(:,15:16)) %y(:,15:16)

disp(xend(end))
disp(yend(end))
disp(y(end,2)-y(end,15))
disp(y(end,3)-y(end,17))
disp(y(end,4)-y(end,18))
% plot states, forces, and states against reference for simscape model
% plot_states(y_simscape(:,1),y_simscape(:,2:10));
% plot_forces(y_simscape(:,1),y_simscape(:,11),y_simscape(:,12),y_simscape(:,13),y_simscape(:,14));
% plot_reference(y_simscape(:,1),y_simscape(:,2:4),y_simscape(:,15:18))
% 
% % solve forward kinematics and plot end effector position for simscape model
% [~,~,~,~,xend,yend] = ForwardKinematics(y_simscape(:,2:4),sysParams);
% plot_endeffector([xend yend],y_simscape(:,15:16)) %y(:,15:16)

function plot_forces(t,u,t1,t2,fc)
    figure('Position',[500,100,800,800]);
    plot(t,u,'k-',t,t1,'b-',t,t2,'g-',t,fc,'m-','LineWidth',2);
    legend("u","$\tau_1$","$\tau_2$","$f_c$","Interpreter","latex");
end

function plot_states(t,x,flag,refs)
labels= ["$\theta_0$","$\theta_1$","$\theta_2$","$\dot{\theta}_0$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
    "$\ddot{\theta}_0$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
switch flag
    case "position"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 1:numState-6
            nexttile
            if i == 1
                plot(t,x(:,i),'b-',t,refs(:,i),'k:','LineWidth',2);
            elseif i == 2
                plot(t,x(:,i),'b-',t,refs(:,i+1),'k:','LineWidth',2);
            elseif i == 3
                plot(t,x(:,i),'b-',t,refs(:,i+1),'k:','LineWidth',2);
            end
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-6
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Reference","Location","eastoutside","FontName","Arial");
    case "velocity"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 4:numState-3
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-3
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Location","eastoutside","FontName","Arial");
    case "acceleration"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 7:numState
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
end
end

function plot_endeffector(x,refs)
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'LineWidth',2);
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'x','LineWidth',2);
    end
    axis padded
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');

end
