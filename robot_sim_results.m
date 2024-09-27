close all; clear; clc;

% Mass-Spring-Damper-Pendulum Dynamics System Parameters
sysParams = params_system();
ctrlParams = params_control();
tSpan = [0,5];
% tSpan = 0:0.01:5;

y = robot_simulation(tSpan, sysParams, ctrlParams);

% out = sim("robot_model.slx");
% t_sim = out.tout;
% th0_sim = out.yout{1}.Values.Data;
% th1_sim = out.yout{2}.Values.Data;
% th2_sim = out.yout{3}.Values.Data;
% xend_sim = out.yout{4}.Values.Data;
% yend_sim = out.yout{5}.Values.Data;

plot_states(y(:,1),y(:,2:10));
plot_forces(y(:,1),y(:,11),y(:,12),y(:,13));
plot_reference(y(:,1),y(:,2:4),y(:,14:17))

[~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:4),sysParams);
plot_endeffector([xend yend],y(:,14:15))

% plot_states(t_sim,[th0_sim th1_sim th2_sim])
% plot_endeffector([xend_sim yend_sim],0)

function plot_forces(t,u,t1,t2)
    figure('Position',[500,100,800,800]);
    plot(t,u,'k-',t,t1,'b-',t,t2,'g-','LineWidth',2);
    legend("u","$\tau_1$","$\tau_2$","Interpreter","latex");
end

function plot_states(t,x)
    refClr = "blue";
    predClr = "red";
    labels= ["$\theta_0$","$\theta_1$","$\theta_2$","$\dot{\theta}_0$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
        "$\ddot{\theta}_0$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    numState = size(x);
    numState = numState(2);
    for i = 1:numState
        nexttile
        plot(t,x(:,i),'Color',refClr,'LineWidth',2);
        hold on
        % xline(1,'k--','LineWidth',2);
        ylabel(labels(i),"Interpreter","latex");
        if i == numState
            xlabel("Time (s)");
        end
        set(get(gca,'ylabel'),'rotation',0);
        set(gca, 'FontSize', 15);
        set(gca, 'FontName', 'Arial');
    end 
end

function plot_reference(t,x,refs)
    refClr = "blue";
    predClr = "red";
    labels= ["$\theta_0$","$\theta_1$","$\theta_2$"];
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    numState = size(x);
    numState = numState(2);
    for i = 1:numState
        nexttile
        if i == 1
            plot(t,x(:,i),'m',t,refs(:,i),'b','LineWidth',2);
        elseif i == 2 
            plot(t,x(:,i),t,refs(:,i+1),'LineWidth',2);
        elseif i == 3
            plot(t,x(:,i),t,refs(:,i+1),'LineWidth',2);
        end
        hold on
        % xline(1,'k--','LineWidth',2);
        ylabel(labels(i),"Interpreter","latex");
        if i == numState
            xlabel("Time (s)");
        end
        set(get(gca,'ylabel'),'rotation',0);
        set(gca, 'FontSize', 15);
        set(gca, 'FontName', 'Arial');
    end 
end

function plot_endeffector(x,refs)
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'LineWidth',2);
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'LineWidth',2);
    end
    axis([-1 3 -1 3])
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');

end
