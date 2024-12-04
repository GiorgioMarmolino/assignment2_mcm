%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix
            
            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);
            re = bTe(1:3, 4);
            for i = 1:self.gm.jointNumber
                bTi = eye(4);
                for j = 1:i
                    bTi = bTi * self.gm.iTj(:, :, j);
                end
                r = bTi(1:3, 4);
                k = bTi(1:3, 3);

                if self.gm.jointType(i) == 0
                    self.J(1:3, i) = k;
                    self.J(4:6, i) = cross(k, (re - r));
                elseif self.gm.jointType(i) == 1
                    self.J(1:3, i) = [0; 0; 0];
                    self.J(4:6, i) = k;
                else
                    error('Joint type not valid');
                end
            end

            
        end
    end
end

