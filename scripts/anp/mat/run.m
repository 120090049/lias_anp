% Example script to call CTOA_GTRS function

% Define the matrix

A = [
-1.,         -1.12177998,  0.,          0.;
-0.87287492, -0.49885371,  0.,          0.;
1.47967799, -0.7349856,   0.02112648,  0.;
0.,          0.,          0.,          1.];

b = [0.; 0.92492352; -4.60409463;  8.50177479];



% Call the CTOA_GTRS function
pos = CTOA_GTRS(A, b);

% Display the results
disp('Estimated position using GTRS:');
disp(pos);
