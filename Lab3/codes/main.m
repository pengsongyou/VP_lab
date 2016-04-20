%% Part1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

f=80; % focal length for both camera. Unit: mm
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
EX2 = inv([R2,T2;0,0,0,1]); % 4*4 extrinsic matrix for camera2 (Transfer from world coordinate to image2 coordinate)

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

% [m,d] = epi_plot(v2,v1,F,[0,height2],[0,400]);
%

% Draw the epipole (way1, the last column of U and V in SVD(F))
[u,s,v] = svd(F);
pr = [u(1,3)/u(3,3),u(2,3)/u(3,3)];
pl = [v(1,3)/v(3,3),v(2,3)/v(3,3)];
% Draw the epipole (way2, projecting the focal point of each camera to the
% image plane of the other)
% Be careful, we should add intrinsic matrix to change to the pixel instead
% of metric
Tr1 = IN1 * EX1; % world point to frame 1
Tr2 = IN2 * EX2; % world point to frame 2

pr_tmp = Tr2 * [0;0;0;1];
pl_tmp = Tr1 * [tx;ty;tz;1];
pr = [pr_tmp(1)/pr_tmp(3),pr_tmp(2)/pr_tmp(3)];
pl = [pl_tmp(1)/pl_tmp(3),pl_tmp(2)/pl_tmp(3)];

% plot(pr(1),pr(2),'b*');
% epi_plot(v1,v2,F',[0,height1],[-340, width1]);

% % Draw the epipole
% pole1 = [(d(2) - d(1))/(m(1) - m(2)),m(1) * (d(2) - d(1))/(m(1) - m(2)) + d(1)];
% plot(pole1(1),pole1(2),'*','MarkerSize',10);
% text(pole1(1)-20,pole1(2)-10, ' epipole','FontSize',10);
% 
% hold off;

%% Step11 Add Gaussian Noise to 2D points
std_noise = 0.05;% Satisfy the condition that 95% of noise points are within the range of [-1,1]

vn1 = v1(:,1:8);
vn1(1:2,:) = vn1(1:2,:) + std_noise * randn(2,size(vn1,2)); % Get noisy 2D points

vn2 = v2(:,1:8);
vn2(1:2,:) = vn2(1:2,:) + std_noise * randn(2,size(vn2,2)); % Get noisy 2D points
% [m,d] = epi_plot(vn2,vn1,F,[0,height2],[0,400]);

%% Repeat Step8 up to 10 with the noisy 2D points

% Step8 : Compute fundamental matrix
F_n = compute_F(vn1,vn2);

[m,d] = epi_plot(vn2,vn1,F_n,[0,height2],[0,400]);
mean_dis = computeMeanDis(vn2,m,d)

% epi_plot(vn1,vn2,F_n',[0,height1],[-340, width1]);

%% Part2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Step14 Fundamental Matrix by SVD, Compare
F_svd = compute_F_svd(v1,v2);

% epi_plot(v2,v1,F_svd,[0,height2],[0,400]); 

F_n_svd = compute_F_svd(vn1,vn2);

% epi_plot(vn2,vn1,F_n_svd,[0,height2],[0,400]); 

%% Plot system

figure;
ps_idx = 1;
ps_3D = V(1:3,ps_idx);
% ps_3D = V(1:3,1:2);
% plot a 3D point
scatter3(ps_3D(1,:),ps_3D(2,:),ps_3D(3,:)); hold on; title('Epipolar system');
xlabel('x'); ylabel('y'); zlabel('z')

% plot the focal points of both camera
o_c1 = [0;0;0;1];% origin of camera 1
x_c1 = [1;0;0;0];% x axis of camera 1
y_c1 = [0;1;0;0];% y axis of camera 1
z_c1 = [0;0;1;0];% z axis of camera 1

plot3(o_c1(1)+[0, 100*x_c1(1), nan, 0, 100*y_c1(1), nan, 0, 100*z_c1(1)], o_c1(2)+[0, 100*x_c1(2), nan, 0, 100*y_c1(2), nan, 0, 100*z_c1(2)],o_c1(3)+[0, 100*x_c1(3), nan, 0, 100*y_c1(3), nan, 0, 100*z_c1(3)] );
t_c1 = text(o_c1(1), o_c1(2), o_c1(3), '\leftarrow camera 1','FontSize',10);

% Plot the image plane of camera 1

cp = [0 256 256 0;0 0 256 256];

cp1_w = im2world(cp,R1,T1,f,uo1,vo1,au1,av1);

plane_x = [cp1_w(1,:),cp1_w(1,1)];
plane_y = [cp1_w(2,:),cp1_w(2,1)];
plane_z = [cp1_w(3,:),cp1_w(3,1)];
plot3(plane_x,plane_y,plane_z);

% plot the project point in the image plane of camera 1
ps_2D1 = v1(1:2,ps_idx);

ps_2D1_w = im2world(ps_2D1,R1,T1,f,uo1,vo1,au1,av1);

scatter3(ps_2D1_w(1,:),ps_2D1_w(2,:),ps_2D1_w(3,:),'r+');

% plot3([0,ps_3D(1,:)],[0,ps_3D(2,:)],[0,ps_3D(3,:)]);


% Plot camera 2 system


o_c2 = [tx;ty;tz;1];% origin of camera 2
x_c2 = R2*[1;0;0];% x axis of camera 2 (transform to world coordinate)
y_c2 = R2*[0;1;0];% y axis of camera 2 
z_c2 = R2*[0;0;1];% z axis of camera 2

plot3(o_c2(1)+[0, 100*x_c2(1), nan, 0, 100*y_c2(1), nan, 0, 100*z_c2(1)], o_c2(2)+[0, 100*x_c2(2), nan, 0, 100*y_c2(2), nan, 0, 100*z_c2(2)],o_c2(3)+[0, 100*x_c2(3), nan, 0, 100*y_c2(3), nan, 0, 100*z_c2(3)] );
t_c2 = text(o_c2(1), o_c2(2), o_c2(3), '\leftarrow camera 2','FontSize',10);


% Plot the image plane of camera 2

cp = [0 256 256 0;0 0 256 256];% Corner point of the image plane

cp_w = im2world(cp,R2,T2,f,uo2,vo2,au2,av2);

plane2_x = [cp_w(1,:),cp_w(1,1)];
plane2_y = [cp_w(2,:),cp_w(2,1)];
plane2_z = [cp_w(3,:),cp_w(3,1)];

plot3(plane2_x,plane2_y,plane2_z);



% plot the project point in the image plane of camera 2
ps_2D2 = v2(1:2,ps_idx);

ps_2D2_w = im2world(ps_2D2,R2,T2,f,uo2,vo2,au2,av2);
scatter3(ps_2D2_w(1,:),ps_2D2_w(2,:),ps_2D2_w(3,:),'r+');

% plot epipoles for both camera
pl_w = im2world(pl',R1,T1,f,uo1,vo1,au1,av1);
pr_w = im2world(pr',R2,T2,f,uo2,vo2,au2,av2);

plot3(pl_w(1),pl_w(2),pl_w(3),'b*');
plot3(pr_w(1),pr_w(2),pr_w(3),'b*');


% Plot the pi plane
for i = 1 : length(ps_idx)
    plot3([o_c1(1),o_c2(1),ps_3D(1,i),o_c1(1)],[o_c1(2),o_c2(2),ps_3D(2,i),o_c1(2)],[o_c1(3),o_c2(3),ps_3D(3,i),o_c1(3)],'-');
end

% % Plot epipolar lines
% for i = 1 : length(ps_idx)
%     el1 = im2world([0;d(i)],R2,T2,f,uo2,vo2,au2,av2);
%     el2 = im2world([400;400*m(i) + d(i)],R2,T2,f,uo2,vo2,au2,av2);
%     plot3([el1(1),el2(1)],[el1(2),el2(2)],[el1(3),el2(3)],'g-','LineWidth',1);
% end

