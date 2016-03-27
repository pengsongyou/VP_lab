% This function is used to compute the transformation matrix using the
% method of Faugeras
function A_F = computeTrans_F(ps_3D, ps_2D)
X = Faugeras(ps_3D,ps_2D); % Apply 11 point method to compute vector X
T1 = X(1:3)';
T2 = X(4:6)';
T3 = X(7:9)';
C1 = X(10);
C2 = X(11);

% Extract the intrinsic and extrinsic parameters
u0_F = T1*T2' / norm(T2,2)^2;
v0_F = T2*T3' / norm(T2,2)^2;
au_F = norm(cross(T1',T2'),2) / norm(T2,2)^2;
av_F = norm(cross(T2',T3'),2) / norm(T2,2)^2;
Tx_F = (norm(T2,2) / norm(cross(T1',T2'),2)) * (C1 - T1*T2'/norm(T2,2)^2);
Ty_F = (norm(T2,2) / norm(cross(T2',T3'),2)) * (C2 - T2*T3'/norm(T2,2)^2);
Tz_F = 1/norm(T2,2);
r1_F = (norm(T2,2) / norm(cross(T1',T2'),2)) .* (T1 - (T1*T2'./norm(T2,2)^2)*T2);
r2_F = (norm(T2,2) / norm(cross(T2',T3'),2)) .* (T3 - (T2*T3'./norm(T2,2)^2)*T2);
r3_F = T2 ./ norm(T2,2);

% Reconstruct Intrinsic and Extrinsic matrix
IN_F = [au_F, 0, u0_F, 0;
        0, av_F, v0_F, 0;
        0, 0, 1, 0];

EX_F = [r1_F,Tx_F;
        r2_F,Ty_F;
        r3_F,Tz_F;
        0,0,0,1];

A_F = IN_F * EX_F;
A_F = A_F ./ A_F(3,4);
end