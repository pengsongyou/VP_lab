% The function is used to project the 3D points to the image plane using
% the known transformation matrix A
function ps_2D = project3Dto2D(A,ps_3D) 
pn = size(ps_3D,2);
ps_2D_b = A * [ps_3D;ones(1,pn)];% Before normalization
ps_2D = zeros(2,pn);
for i = 1 : pn
    ps_2D(1,i) = ps_2D_b(1,i)/ps_2D_b(3,i);
    ps_2D(2,i) = ps_2D_b(2,i)/ps_2D_b(3,i);
end    
end