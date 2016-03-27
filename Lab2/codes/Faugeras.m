% This function is used to compute the X vector of the method of Faugeras
function X = Faugeras(ps_3D,ps_2D)
pn = size(ps_3D,2); % point number
Q = [];
B = [];
for i = 1 : pn
    Q = [Q; ps_3D(:,i)', -ps_3D(:,i)' .* ps_2D(1,i), 0,0,0,1,0; 0,0,0, -ps_3D(:,i)' .* ps_2D(2,i),ps_3D(:,i)',0,1];
    B = [B;ps_2D(1,i);ps_2D(2,i)];
end
X = Q\B; % Least mean square
end