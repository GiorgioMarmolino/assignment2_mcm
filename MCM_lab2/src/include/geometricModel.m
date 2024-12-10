%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end

        %% METHOD - UPDATE DIRECT GEOMETRY
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            
            if length(q) ~= self.jointNumber
                error('q length not valid')
            end

            self.q = q;
            for i = 1:self.jointNumber

                Ti_0 = self.iTj_0(:, :, i);

                if self.jointType(i) == 0 % Revolut Joint case
                    R = [cos(q(i)), -sin(q(i)), 0;
                         sin(q(i)), cos(q(i)),  0;
                         0,         0,          1];
                    self.iTj(:, :, i) = Ti_0 * [R, [0; 0; 0]; 0, 0, 0, 1];

                elseif self.jointType(i) == 1 % Prismatic Joint case
                    
                    T = [0; 0; q(i)];
                    self.iTj(:, :, i) = Ti_0 * [eye(3, 3), T; 0, 0, 0, 1];

                else
                    error('Joint type not valid'); %error case
                end
            end

            self.iTj(abs(self.iTj) < 1e-6) = 0;
            self.iTj(abs(self.iTj - 1) < 1e-6) = 1;
            self.iTj(abs(self.iTj + 1) < 1e-6) = -1;
        end





        %% METHOD - GET TRANSFORMATION WRTBASE 
        function [bTk] = getTransformWrtBase(self,k)
            
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            if k > self.jointNumber || k < 1
                error('K not valid');
            end

            bTk = eye(4);

            for i = 1:k
                bTk = bTk * self.iTj(:, :, i);
            end

            bTk(abs(bTk) < 1e-6) = 0;
            bTk(abs(bTk - 1) < 1e-6) = 1;
            bTk(abs(bTk + 1) < 1e-6) = -1;
        end






        %% METHOD - GET TRANSFORMATION WRT
        function [sTk] = getTransformWrt(self, s, k)
             
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            if k > self.jointNumber || k < 1
                error('K not valid');
            end
            if s > k || s < 1
                error('s not valid');
            end

            sTk = eye(4);

            for i = (s+1):k
                sTk = sTk * self.iTj(:, :, i);
            end
        end

    end
end


