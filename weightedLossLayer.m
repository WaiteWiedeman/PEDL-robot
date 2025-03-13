classdef weightedLossLayer < nnet.layer.RegressionLayer ...
        & nnet.layer.Acceleratable
    % Example custom regression layer with mean-absolute-error loss.
    
    methods
        function layer = weightedLossLayer(name)
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
            f = physics_law(Y,sysParams);
            fTarget = physics_law(T,sysParams);
            physicLoss = mse(f, fTarget);
            
            % End Effector loss
            [~,~,~,~,xend1,yend1,xend2,yend2] = ForwardKinematics(Y(1:3),sysParams);
            [~,~,~,~,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(T(1:3),sysParams);
            endEff = [xend1;yend1;xend2;yend2];
            endEffTarget = [xendTarget1;yendTarget1;xendTarget2;yendTarget2];
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