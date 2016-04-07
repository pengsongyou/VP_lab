%% Part1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

%% Step1 Define camera 1
au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;
height1 = 256;
width1 = 256;

R1 = eye(3,3); 
T1 = zeros(3,1);

%% Step2 Define camera 2
au2 = 90; av2 = 110; uo2 = 128; vo2 = 128;
ax = 0.1; by = pi/4; cz = 0.2; tx = -1000; ty = 190; tz = 230;
height2 = 256;
width2 = 256;

RotX = [1, 0, 0; 0, cos(ax), -sin(ax); 0, sin(ax), cos(ax)];
RotY = [cos(by), 0, sin(by); 0, 1, 0; -sin(by), 0, cos(by)];
RotZ = [cos(cz), -sin(cz), 0; sin(cz), cos(cz), 0; 0, 0, 1];
R2 = RotX * RotY * RotZ;
T2 = [tx;ty;tz];

%% Step3 Get the intrinsic matrices for both cameras
A1 = [au1, 0, uo1;
       0, av1, vo1;
       0, 0, 1];
  
A2 = [au2, 0, uo2;
       0, av2, vo2;
       0, 0, 1];  
   
%% Step4 Get the Fundamental matrix
T_cross = [0, -T2(3), T2(2);
           T2(3), 0, -T2(1);
           -T2(2), T2(1), 0];
           
F_ground = inv(A2') * R2' * T_cross * inv(A1);
F_ground = F_ground./ F_ground(9);
%% Step5 Define Object point
pn = 20;
V = zeros(4,pn);
V(:,1) = [100;-400;2000;1];
V(:,2) = [300;-400;3000;1];
V(:,3) = [500;-400;4000;1];
V(:,4) = [700;-400;2000;1];
V(:,5) = [900;-400;3000;1];
V(:,6) = [100;-50;4000;1];
V(:,7) = [300;-50;2000;1];
V(:,8) = [500;-50;3000;1];
V(:,9) = [700;-50;4000;1];
V(:,10) = [900;-50;2000;1];
V(:,11) = [100;50;3000;1];
V(:,12) = [300;50;4000;1];
V(:,13) = [500;50;2000;1];
V(:,14) = [700;50;3000;1];
V(:,15) = [900;50;4000;1];
V(:,16) = [100;400;2000;1];
V(:,17) = [300;400;3000;1];
V(:,18) = [500;400;4000;1];
V(:,19) = [700;400;2000;1];
V(:,20) = [900;400;3000;1];

%% Step 6 Compute the image points in both image planes

% Define intrinsic and extrinsic matrix first
IN1 = [A1,[0;0;0]];% 3*4 intrinsic matrix for camera1
IN2 = [A2,[0;0;0]];% 3*4 intrinsic matrix for camera2
EX1 = [R1,T1;0,0,0,1];% 4*4 extrinsic matrix for camera1
EX2 = inv([R2,T2;0,0,0,1]); % 4*4 extrinsic matrix for camera2

% Compute the 2D points in both image planes
v1 = zeros(3,pn);
v2 = zeros(3,pn);
for i = 1 : pn
   v1(:,i) = IN1 * EX1 * V(:,i);
   v1(:,i) = v1(:,i) ./ v1(3,i);
   v2(:,i) = IN2 * EX2 * V(:,i);
   v2(:,i) = v2(:,i) ./ v2(3,i);
end

%% Step 7 Draw the 2D points in two camera windows

%% Step8 Compute Fundamental matrix using 8-pionts least mean square method
F = compute_F(v1,v2);

%% Step9 Compare the F acquired from Step8 with the Ground truth
F == F_ground

%% Step10 Draw epipoles and epipolar lines

% Step 7 is moved here
% 

epi_plot(v2,v1,F,[0,height2],[0,400]);

% Draw the epipole
% pole2 = [(d(2) - d(1))/(m(1) - m(2)),m(1) * (d(2) - d(1))/(m(1) - m(2)) + d(1)];
% plot(pole2(1),pole2(2),'*','MarkerSize',10);
% text(pole2(1)-20,pole2(2)-10, ' epipole','FontSize',10);


epi_plot(v1,v2,F',[0,height1],[-340, width1]);

% % Draw the epipole
% pole1 = [(d(2) - d(1))/(m(1) - m(2)),m(1) * (d(2) - d(1))/(m(1) - m(2)) + d(1)];
% plot(pole1(1),pole1(2),'*','MarkerSize',10);
% text(pole1(1)-20,pole1(2)-10, ' epipole','FontSize',10);
% 
% hold off;

%% Step11 Add Gaussian Noise to 2D points
std_noise = 0.5;% Satisfy the condition that 95% of noise points are within the range of [-1,1]

vn1 = v1;
vn1 = vn1 + std_noise * randn(3,pn); % Get noisy 2D points

vn2 = v2;
vn2 = vn2 + std_noise * randn(3,pn); % Get noisy 2D points


%% Repeat Step8 up to 10 with the noisy 2D points

% Step8 : Compute fundamental matrix
F_n = compute_F(vn1,vn2);

epi_plot(vn2,vn1,F_n,[0,height2],[0,400]);

epi_plot(vn1,vn2,F_n',[0,height1],[-340, width1]);

%% Part2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step14 Fundamental Matrix by SVD, Compare
F_svd = compute_F_svd(v1,v2);

epi_plot(v2,v1,F_svd,[0,height2],[0,400]); 

F_n_svd = compute_F_svd(vn1,vn2);

epi_plot(vn2,vn1,F_n_svd,[0,height2],[0,400]); 