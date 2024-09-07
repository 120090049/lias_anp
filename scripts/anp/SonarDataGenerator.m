classdef SonarDataGenerator
    properties
        P_W          % 3D world points
        R_SW         % Rotation matrix
        t_S          % Translation vector
        Var_Noise    % Noise variance
        n            % Number of points
    end
    
    methods
        function obj = SonarDataGenerator(P_W, R_SW, t_S, Var_Noise)
            % Constructor for SonarDataGenerator
            if nargin < 4
                Var_Noise = 1.0;  % Default noise variance if not provided
            end
            obj.P_W = P_W;
            obj.R_SW = R_SW;
            obj.t_S = t_S;
            obj.Var_Noise = Var_Noise;
            obj.n = size(P_W, 2);  % Number of points (columns in P_W)
        end
        
        function [P_S, P_SI, P_SI_Noise] = generate_data(obj)
            % Initialize output matrices
            P_S = zeros(3, obj.n);        % 3D sonar frame points
            d = zeros(1, obj.n);          % Distances
            cos_theta = zeros(1, obj.n);  % Cosine of theta
            sin_theta = zeros(1, obj.n);  % Sine of theta
            tan_theta = zeros(1, obj.n);  % Tangent of theta
            theta = zeros(1, obj.n);      % Theta values
            cos_phi = zeros(1, obj.n);    % Cosine of phi
            P_SI = zeros(2, obj.n);       % Sonar image points (2D)
            
            % Loop through each point and calculate values
            for i = 1:obj.n
                P_S(:, i) = obj.R_SW * obj.P_W(:, i) + obj.t_S;
                d(i) = norm(P_S(:, i));   % Distance calculation
                cos_theta(i) = P_S(1, i) / sqrt(P_S(1, i)^2 + P_S(2, i)^2);
                sin_theta(i) = P_S(2, i) / sqrt(P_S(1, i)^2 + P_S(2, i)^2);
                tan_theta(i) = sin_theta(i) / cos_theta(i);
                theta(i) = atan(tan_theta(i));
                cos_phi(i) = sqrt(P_S(1, i)^2 + P_S(2, i)^2) / d(i);
                P_SI(1, i) = d(i) * cos_theta(i);
                P_SI(2, i) = d(i) * sin_theta(i);
            end
            
            % Add noise to P_SI
            P_SI_Noise = P_SI + obj.Var_Noise * randn(2, obj.n);
        end
    end
end
