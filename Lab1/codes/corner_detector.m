close all;
clear;
clc;

chess3 = imread('../images/chessboard03.png'); 
chess4 = imread('../images/chessboard04.png'); 
chess5 = imread('../images/chessboard05.png'); 
chess6 = imread('../images/chessboard06.png'); 

% Usage of the harris function:
% The fisrt parameter is the image, the second one is size of Gaussian
% filter

% chessboard 3
subp = harris(chess3, 3);
figure; imshow(chess3,'InitialMagnification','fit');hold on;title('chessboard3')
for i=1:size(subp,1),
    plot(subp(i,2), subp(i,1), 'g+');
end
hold off;

% chessboard 4
subp = harris(chess4, 9);
figure; imshow(chess4,'InitialMagnification','fit');hold on;title('chessboard4')
for i=1:size(subp,1),
    plot(subp(i,2), subp(i,1), 'g+');
end
hold off;

% chessboard 5
subp = harris(chess5, 9);
figure; imshow(chess5,'InitialMagnification','fit');hold on;title('chessboard5')
for i=1:size(subp,1),
    plot(subp(i,2), subp(i,1), 'g+');
end
hold off;

% chessboard 6
subp = harris(chess6, 9);
figure; imshow(chess6,'InitialMagnification','fit');hold on;title('chessboard6')
for i=1:size(subp,1),
    plot(subp(i,2), subp(i,1), 'g+');
end
hold off;