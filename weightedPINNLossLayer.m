classdef weightedPINNLossLayer < nnet.layer.RegressionLayer ...
        & nnet.layer.Acceleratable
    % Example custom regression layer with mean-absolute-error loss.
    
    methods
        function layer = weightedPINNLossLayer(name)
            % layer = weightedLossLayer(name) creates a
            % mean-sqaure-error regression layer and specifies the layer
            % name.
            % Set layer name.
            layer.Name = name;
            % Set layer description.
            layer.Description = 'Weighted Loss Layer';
        end
        
        function loss = forwardLoss(layer, Y, T)
            % Inputs:
            %         layer - Output layer
            %         Y     – Predictions made by network
            %         T     – Training targets
            % data loss: compute the difference between target and predicted values
            dataLoss = mse(Y, T);
            
            % physics loss
            sysParams = params_system();
            ctrlParams = params_control();
            fTarget = physics_law(T,sysParams);

            fc = coulomb_friction(Y(4), sysParams, ctrlParams.friction);
            xdot = robot_xdot(Y([1 4 2 5 3 6]), fTarget, fc, sysParams);
            f = physics_law([Y(1:6) xdot([2,4,6])],sysParams);

            physicLoss = mse(f, fTarget);

            % End Effector loss
            [~,~,~,~,xend,yend] = ForwardKinematics(Y(1:3),sysParams);
            [~,~,~,~,xendTarget,yendTarget] = ForwardKinematics(T(1:3),sysParams);
            endEff = [xend;yend];
            endEffTarget = [xendTarget;yendTarget];
            endEffloss = mse(endEff,endEffTarget);

            % final loss, combining data loss and physics loss
            trainParams = params_training();
            loss = (1.0-trainParams.alpha-trainParams.beta)*dataLoss + trainParams.alpha*physicLoss + trainParams.beta*endEffloss;
        end

        function dLdY = backwardLoss(layer,Y,T)
            % (Optional) Backward propagate the derivative of the loss 
            % function.
            %
            % Inputs:
            %         layer - Output layer
            %         Y     – Predictions made by network
            %         T     – Training targets
            %
            % Output:
            %         dLdY  - Derivative of the loss with respect to the 
            %                 predictions Y        
            dLdY = 2*(Y-T)/numel(T);
        end
    end
end