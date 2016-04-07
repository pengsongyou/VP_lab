function F = compute_F(v1,v2)
pn = size(v1,2);
U = zeros(pn,8);
for i = 1 : pn
    x = v1(1,i);
    x_ = v2(1,i);
    y = v1(2,i);
    y_ = v2(2,i);
    U(i,:) = [x*x_, x_*y, x_, x*y_, y*y_, y_, x, y];
end
F_v = -U\ones(pn,1);
F = reshape([F_v;1],[3,3])';
end