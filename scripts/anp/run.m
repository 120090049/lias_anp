% MATLAB script to test compute_t_R function

% Initialize parameters
clc
clear

P_W = [30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15;  % Example data
       44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42;
       35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47];
R_SW = [-0.5798, 0.4836, -0.6557;
        -0.8135, -0.3883, 0.4329;
        -0.0453, 0.7844, 0.6186];
t_S = [6; 4; 8];

% Create SonarDataGenerator object
generator = SonarDataGenerator(P_W, R_SW, t_S);
[P_S, P_SI, P_SI_Noise] = generator.generate_data();


% P_SI = [-34.28415699, -23.4147263,   30.58719872, -71.36205414, -62.98350553,  -83.66340331, -54.11706308, -27.53627976, -35.66165012, -33.8476791,  -67.36441633;
%         -58.46019646, -46.09452045, -67.72413967,  15.58309519,   5.06912889,  -47.81114677, -18.30154963, -67.59046453, -65.16200224, -73.65466778,  -21.42185885;];



% Call the compute_t_R function
% [t_s_cal, R_sw_cal] = compute_t_R_test(P_SI, P_W);
[R_sw_cal, t_s_cal] = compute_t_R(P_W, P_SI_Noise,0,0);

% Display results
disp('t_s_cal:');
disp(t_s_cal);

disp('R_sw_cal:');
disp(R_sw_cal);