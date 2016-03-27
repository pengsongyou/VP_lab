% This function is used to compute the transformation matrix using the
% method of Hall
function A = computeTrans_H(ps_3D, ps_2D)
Q =[];
B = [];
pn = size(ps_2D,2);
for i = 1 : pn
    Q = [Q;ps_3D(1,i), ps_3D(2,i), ps_3D(3,i),1,0,0,0,0,-ps_2D(1,i)*ps_3D(1,i),-ps_2D(1,i)*ps_3D(2,i),-ps_2D(1,i)*ps_3D(3,i);
        0,0,0,0,ps_3D(1,i), ps_3D(2,i), ps_3D(3,i),1,-ps_2D(2,i)*ps_3D(1,i),-ps_2D(2,i)*ps_3D(2,i),-ps_2D(2,i)*ps_3D(3,i)];
    B = [B;ps_2D(1,i);ps_2D(2,i)];
end
A_vec = Q\B;% Least mean square method
A_vec = [A_vec;1];
A = reshape(A_vec,[4,3]);
A = A';
end