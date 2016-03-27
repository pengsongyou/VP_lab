%% Part1 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clc;
clear;
%% Step1: Define intrinsic parameters
au = 557.093; av=712.9824; % Pixel conversion parameters
u0=326.3819; v0=298.6679; % Principal point in the image plane
f=80; % focal length. Unit: mm
Tx=100; Ty=0; Tz=1500 ;% Translation parameters. Unit: mm
Phix=0.8*pi/2; Phiy=-1.8*pi/2; Phix1=pi/5; % Rotation parameters
height = 480; width = 640; % Image size

%% Step2: Intrinsic and extrinsic transformation matrices
IN = [au, 0, u0, 0;
      0, av, v0, 0;
      0, 0, 1, 0];
  
RotX = [1, 0, 0; 0, cos(Phix), -sin(Phix); 0, sin(Phix), cos(Phix)];
RotY = [cos(Phiy), 0, sin(Phiy); 0, 1, 0; -sin(Phiy), 0, cos(Phiy)];
RotX1 = [1, 0, 0; 0, cos(Phix1), -sin(Phix1); 0, sin(Phix1), cos(Phix1)];
R = RotX * RotY * RotX1;
T = [Tx;Ty;Tz];
EX = [R,T;0,0,0,1];  

%% Step3: Define 3D points
pn = 6; % point number
ps_3D = randi([-480,480],3,pn);% 3D point set w.r.t world coordinate

%% Step4: Project 3D points to image plane
ps_2D = project3Dto2D(IN*EX, ps_3D); % 2D point set w.r.t image coordinate

%% Step5: Check the points in the image plane
figure; hold on; title('2D points in image plane');
for i = 1 : pn
    plot(ps_2D(2,i),ps_2D(1,i),'r+', 'MarkerSize',10,'linewidth',10);    
end
hold off
%% Step6: Use the method of Hall to acquire transformation matrix
A = computeTrans_H(ps_3D, ps_2D);

%% Step7: Compare A and IN*EX
A_ground = IN*EX; % Ground truth of the transformation
A_ground = A_ground./A_ground(3,4); % Normalize the A_ground with A_34

dif = norm(A_ground - A);
disp('The difference between the ground truth matrix');
disp(['and matrix obtained from method of Hall is: ',num2str(dif)]);
disp(' ');

%% Step8: Add noise to 95% 2D points and compute transformation matrix
std_noise = 0.5;% Satisfy the condition that 95% of noise points are within the range of [-1,1] or [-2,2] or [-3,3]
noise = std_noise * randn(2,pn); % Create noise

nps_2D = ps_2D;
nps_2D = nps_2D + noise; % Get noisy 2D points

A_noise_H = computeTrans_H(ps_3D, nps_2D); % Compute the transformation matrix using Hall

noisy_2D = project3Dto2D(A_noise_H,ps_3D);% noisy 2D points after projection

accuracy_Hall = mean(mean(abs(noisy_2D - ps_2D))) % Check the accuracy by get the mean difference
%% Part2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step 10 Compute X using the points without noise
A_F = computeTrans_F(ps_3D, ps_2D);% Use method of Faugeras to get transformation method
ps_2D_F = project3Dto2D(A_F,ps_3D);

%% Step 11 
A_noise_F = computeTrans_F(ps_3D, nps_2D); % Acquire transforamtion matrix from 2D noisy points
noisy_2D_F = project3Dto2D(A_noise_F,ps_3D); 
accuracy_Faugeras = mean(mean(abs(noisy_2D_F - ps_2D))) % Check the accuracy by get the mean difference

%% Part3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; 

% plot 3D point 
ps_3D_camera = EX * [ps_3D;ones(1,pn)];% transform from world coordinate to camera coordinate
scatter3(ps_3D_camera(1,:)',ps_3D_camera(2,:)',ps_3D_camera(3,:)'); hold on; title('Calibration system');
xlabel('x'); ylabel('y'); zlabel('z')

% plot world coordinate frame with respect to camera coordinate 
o_world = [0;0;0;1];% origin of world
x_world = [1;0;0;0];% x axis of world
y_world = [0;1;0;0];% y axis of world
z_world = [0;0;1;0];% z axis of world

o_world_c = EX*o_world;
x_world_c = EX*x_world;
y_world_c = EX*y_world;
z_world_c = EX*z_world;

plot3(o_world_c(1)+[0, 100*x_world_c(1), nan, 0, 100*y_world_c(1), nan, 0, 100*z_world_c(1)], o_world_c(2)+[0, 100*x_world_c(2), nan, 0, 100*y_world_c(2), nan, 0, 100*z_world_c(2)],o_world_c(3)+[0, 100*x_world_c(3), nan, 0, 100*y_world_c(3), nan, 0, 100*z_world_c(3)] );
t_b = text(o_world_c(1), o_world_c(2), o_world_c(3), '\leftarrow world coordinate','FontSize',10);

% plot camera coordinate
x0 = 0; % Define the origin of camera coordinate is (0,0,0)
y0 = 0;
z0 = 0;
plot3(x0+[0, 100, nan, 0, 0, nan, 0, 0], y0+[0, 0, nan, 0, 100, nan, 0, 0],z0+[0, 0, nan, 0, 0, nan, 0, 100] );
t_a = text(x0, y0, z0, '\leftarrow camera coordinate','FontSize',10);

% plot origin of R coordinate (OR)
% or_c = [0;0;f;1];% origin of R with respect to c

% plot image plane
cp1 = [0;0]; % corner point in image coordinate 
cp2 = [480;0];
cp3 = [480;640];
cp4 = [0;640];

cp = [0 480 480 0;0 0 640 640];
% Transform from image coordinate to R coordiante first
cp_R = bsxfun(@minus,[u0;v0],cp);

% Transform from image coordinate to camera coordinate
ku = -au/f;
kv = -av/f;
cp_c = bsxfun(@rdivide,cp_R,[ku;kv]);

plane_x = [cp_c(1,:),cp_c(1,1)];
plane_y = [cp_c(2,:),cp_c(2,1)];
plane_z = [f,f,f,f,f];
plot3(plane_x,plane_y,plane_z);

% Plot 2D points with respect to camera coordinate
ps_2D_R = bsxfun(@minus,[u0;v0],ps_2D);
ps_2D_c = bsxfun(@rdivide,ps_2D_R, [ku;kv]);

ps_2D_camera = [ps_2D_c;f*ones(1,pn)];
scatter3(ps_2D_camera(1,:),ps_2D_camera(2,:),ps_2D_camera(3,:),'r+');

for i = 1 : pn
    x = [0,ps_3D_camera(1,i)];
    y = [0,ps_3D_camera(2,i)];
    z = [0,ps_3D_camera(3,i)];
    plot3(x,y,z);
end


hold off;