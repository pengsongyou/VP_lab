function cp_w = im2world(cp,R,T,f,uo,vo,au,av)
num = size(cp,2);
cp_R = bsxfun(@minus,[uo;vo],cp);

% Transform from R coordinate to camera coordinate
ku = -au/f;
kv = -av/f;
cp_c = bsxfun(@rdivide,cp_R,[ku;kv]);

% Transform from camera2 coordinate to world coodinate

cp_w = [R,T;0,0,0,1] * [cp_c;f.*ones(1,num);ones(1,num)];

end