function [m,d] = epi_plot(v1,v2,F,lim_h,lim_w)
% Plot points in image plane
figure; hold on; %title('2D points in image plane2');
pn = size(v1,2);
for i = 1 : pn 
    plot(v1(1,i),v1(2,i),'r+', 'MarkerSize',10);    
end

xlim(lim_w)
ylim(lim_h)

% Draw epipolar lines in plane 2
m = zeros(pn,1);
d = zeros(pn,1);
for i = 1 : pn
    
    lm2 = F * v2(:,i);
    m(i) = -lm2(1) / lm2(2);
    d(i) = -lm2(3) / lm2(2);
    
    plot([lim_w(1), lim_w(2)], [m(i) * lim_w(1) + d(i), m(i) * lim_w(2) + d(i)],'-');
end

hold off;
end