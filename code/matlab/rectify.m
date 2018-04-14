clc
clear
%%
isCalibrated = input('开始标定，若为第一次请按1，否则按0');
if isCalibrated == 1
%相机标定
numImagePairs = 30;
for i = 1:numImagePairs
    if i <= 10
        leftPath = ['F:\Documents\Graduation Project\Dataset\chessBoard-1\left_00000',num2str(i-1),'.jpg'];
        rightPath = ['F:\Documents\Graduation Project\Dataset\chessBoard-1\right_00000',num2str(i-1),'.jpg'];
    else
        leftPath = ['F:\Documents\Graduation Project\Dataset\chessBoard-1\left_0000',num2str(i-1),'.jpg'];
        rightPath = ['F:\Documents\Graduation Project\Dataset\chessBoard-1\right_0000',num2str(i-1),'.jpg'];
    end
    imageFiles1{i} = leftPath;
    imageFiles2{i} = rightPath;
end
im = imread(imageFiles1{1});
imagePoints = detectCheckerboardPoints(im);

images1 = cast([], 'uint8');
images2 = cast([], 'uint8');
for i = 1:numel(imageFiles1)
    im = imread(imageFiles1{i});
    im(3:700, 1247:end, :) = 0;
    images1(:, :, :, i) = im;
    
    im = imread(imageFiles2{i});
    im(1:700, 1198:end, :) = 0;
    images2(:, :, :, i) = im;
end

[imagePoints, boardSize] = detectCheckerboardPoints(images1, images2);
squareSize = 20.65; % millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Compute the stereo camera parameters.
stereoParams = estimateCameraParameters(imagePoints, worldPoints);
disp('标定结束');
figure; showReprojectionErrors(stereoParams);
else
    calib_result = load('CALIB_RESULT.mat');
    stereoParams = calib_result.stereoParams;
end
%%
disp('校正');
%Read in the stereo pair of images.
I1 = imread('F:\Documents\Graduation Project\Dataset\object-0\left_000000.jpg');
I2 = imread('F:\Documents\Graduation Project\Dataset\object-0\right_000000.jpg');

% I1 = imread('left1.bmp');
% I2 = imread('right1.bmp');

% Rectify the images.
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

imwrite(J1,'left_rectified.jpg');
imwrite(J2,'right_rectified.jpg');
figure(1); 
A = [J1,J2];
imshow(A);title('校正后');
hold on
[m,n] = size(A);
M = 12;  % 水平分量
lw = 2;
mx = ones(1,M+1);
my = linspace(1,m,M+1);
% 画水平线
for k = 1:M+1
    line([mx(k) n*mx(k)],[my(k) my(k)],'color','b','LineWidth',lw);
end
figure(2); 
A = [I1,I2];
imshow(A);title('校正前');
hold on
[m,n] = size(A);
M = 10;  % 水平分量
lw = 2;
mx = ones(1,M+1);
my = linspace(1,m,M+1);
% 画水平线
for k = 1:M+1
    line([mx(k) n*mx(k)],[my(k) my(k)],'color','b','LineWidth',lw);
end
%%
%计算视差图
disparityMap = disparity(J1,J2,'Method','SemiGlobal','BlockSize',15);
figure(3);
imshow(disparityMap, [0, 64], 'InitialMagnification', 50);
colormap('jet');
colorbar;
title('Disparity Map');

%%
%计算三维坐标
disparity = medfilt2(disparityMap);
pointCloud = reconstructScene(disparityMap,stereoParams);
pointCloud = pointCloud / 1000;
[reducedColorImage,reducedColorMap] = gray2ind(J1,128);

% Plot the 3D points of each color.
hFig = figure; hold on;
set(hFig, 'Position', [1 1 840   630]);
hAxes = gca;

X = pointCloud(:, :, 1);
Y = pointCloud(:, :, 2);
Z = pointCloud(:, :, 3);

for i = 1:size(reducedColorMap, 1)
    % Find the pixels of the current color.
    x = X(reducedColorImage == i-1);
    y = Y(reducedColorImage == i-1);
    z = Z(reducedColorImage == i-1);
    
    if isempty(x)
        continue;
    end

    % Eliminate invalid values.
    idx = isfinite(x);
    x = x(idx);
    y = -y(idx);
    z = z(idx);
    
    maxZ = 2;
    minZ = 1.4;
     x = x(z > minZ & z < maxZ);
    y = y(z > minZ & z < maxZ);
    z = z(z > minZ & z < maxZ);
    
    plot3(hAxes, x,y,z, '.', 'MarkerEdgeColor', reducedColorMap(i, :));
    xlabel('x');ylabel('y');zlabel('z');
    hold on;
end

%%
% %写入坐标数据
% fid = fopen('points.asc','wt');
% length = size(x);
% for i = 1:length
%     fprintf(fid,'%d\t',x(i));
%     fprintf(fid,'%d\t',y(i));
%     fprintf(fid,'%d\n',z(i));
% end
% fclose(fid);

[m,n] = size(X);
fid1 = fopen('points.asc','wt');
for i = 1:m
    for j = 1:n
        if isnan(X(i,j))+isnan(Y(i,j))+isnan(Z(i,j)) == 0
            if isinf(X(i,j))+isinf(Y(i,j))+isinf(Z(i,j)) == 0
                x = X(i,j);
                y = Y(i,j);
                z = Z(i,j);
                fprintf(fid,'%d\t',x);
                fprintf(fid,'%d\t',y);
                fprintf(fid,'%d\n',z);
            end
        end
    end
end
fclose(fid1);