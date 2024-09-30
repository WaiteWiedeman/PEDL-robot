close all; clear; clc;

% Two-link Planar Robot on Cart Dynamics System Parameters
sysParams = params_system();
ctrlParams = params_control();
tSpan = [0,5]; %0:0.01:5;

% run simulation
y = robot_simulation(tSpan, sysParams, ctrlParams);

% run simscape model
y_simscape = run_simscape();

% plot states, forces, and states against reference
plot_states(y(:,1),y(:,2:10));
plot_forces(y(:,1),y(:,11),y(:,12),y(:,13),y(:,14));
plot_reference(y(:,1),y(:,2:4),y(:,15:18))

% solve forward kinematics and plot end effector position
[~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:4),sysParams);
plot_endeffector([xend yend],y(:,15:16)) %y(:,15:16)

% plot states, forces, and states against reference for simscape model
plot_states(y_simscape(:,1),y_simscape(:,2:10));
plot_forces(y_simscape(:,1),y_simscape(:,11),y_simscape(:,12),y_simscape(:,13),y_simscape(:,14));
plot_reference(y_simscape(:,1),y_simscape(:,2:4),y_simscape(:,15:18))

% solve forward kinematics and plot end effector position for simscape model
[~,~,~,~,xend,yend] = ForwardKinematics(y_simscape(:,2:4),sysParams);
plot_endeffector([xend yend],y_simscape(:,15:16)) %y(:,15:16)

function plot_forces(t,u,t1,t2,fc)
    figure('Position',[500,100,800,800]);
    plot(t,u,'k-',t,t1,'b-',t,t2,'g-',t,fc,'m-','LineWidth',2);
    legend("u","$\tau_1$","$\tau_2$","$f_c$","Interpreter","latex");
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
    axis padded
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');

end
