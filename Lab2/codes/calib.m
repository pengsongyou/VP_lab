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
%RotZ = [cos(Phiz), -sin(Phiz), 0; sin(Phiz), cos(Phiz), 0; 0, 0, 1];
R = RotX * RotY * RotX1;
T = [Tx;Ty;Tz];
EX = [R,T;0,0,0,1];  

%% Step3: Define 3D points
pn = 6; % point number
ps_3D = randi([-480,480],3,pn);% Point set

%% Step4: Project 3D points to image plane
ps_2D_b = IN * EX * [ps_3D;ones(1,pn)];% Before normalization
ps_2D = zeros(2,pn);
for i = 1 : pn
    ps_2D(1,i) = ps_2D_b(1,i)/ps_2D_b(3,i);
    ps_2D(2,i) = ps_2D_b(2,i)/ps_2D_b(3,i);
end
%% Step5: Check the points in the image plane
figure; hold on;
for i = 1 : pn
    plot(ps_2D(1,i),ps_2D(2,i),'r+');    
end
% this part I should try to compare the difference between well spread
% point and some point get together
%% Step6: Method of Hall
A = computeTrans(ps_3D, ps_2D);
%% Step7: Compare A and IN*EX
A_ground = IN*EX; % Ground truth of the transformation
A_ground = A_ground./A_ground(3,4); % Normalize the A_ground with A34
% They are the same when there is no noise

%% Step8: Add noise to 95% 2D points and compute transformation matrix
npn = floor(pn*0.95); % Noisy point number
noise = rand(2,npn);
noise = 2*noise / max(max(noise)) -1; % range [-1,1]
nps_2D = ps_2D;
nps_2D(:,1:npn) = nps_2D(:,1:npn)+ noise; % noisy 2D point set (around 95% noise)
A_noise = computeTrans(ps_3D, nps_2D);