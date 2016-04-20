function mean_dis = computeMeanDis(v,m,d)
a = -m;
b = 1;
c = -d;
dis = abs(a.*v(1,:)' + b.*v(2,:)'+c)./sqrt((a.^2 + b.^2));
mean_dis = mean(dis);
end