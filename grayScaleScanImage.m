clc
for m=1:3891
clearvars -except m
close all
%%
tic;
file_path=('D:\data\ScanIntensity\Scan_for_MI_Intensity_');
scanName = sprintf('%s/SCANS_FOR_MI/Scan_for_MI_%04d.txt','D:\data',m);
M = dlmread(scanName);
R = M(2:size(M,1),4);
imageIndex = M(1,1)+1;
M(1,1) = imageIndex;
pts1 = pointCloud(M(2:size(M,1),1:3));
%pts1=M(2:size(M,1),1:3);
load laser2camera.mat;
%%
 S=pts1.Count;
 intensity = cell(5,3);
 pts2 = cell(5,1);
 int = zeros(S,1);
for i=1:5
    imageName = sprintf('%s/Cam%01d/image%04d.ppm','D:\dataset-2\IJRR-Dataset-2\IMAGES',i-1,imageIndex);
    IM = imread(imageName);
    IM1 = rgb2gray(IM);
    pts11=pctransform(pts1,tform{i});
    pts11=pts11.Location;
%   pts11 = pts1*R_cl{i}'+repmat(T_cl(i,:),size(pts1,1),1);
%   pts12 = pts11(pts11(:,3)>0 & pts11(:,1)<2.5,1:3);
    pts12 = pts11(pts11(:,3)>0,1:3);
    pts13 = pts12*K{i}';
    tempx = round(pts13(:,1)./pts13(:,3));
    tempy = round(pts13(:,2)./pts13(:,3));
    pts2{i,1}=pts1.Location(0<tempx & tempx<1617 & 0<tempy & tempy<617,1:3);
%   pts2{i,1} = pts11(0<tempx & tempx<1617 & 0<tempy & tempy<617,1:3);
    x=tempx(0<tempx & tempx<1617 & 0<tempy & tempy<617);
    y=tempy(0<tempx & tempx<1617 & 0<tempy & tempy<617);
    for j=1:length(x)
    intensity{i,1}(j,1) = IM1(y(j),x(j));
    end
    intensity{i,2} = pts2{i,1};
    A = ismember(pts1.Location,intensity{i,2});
    intensity{i,3} = find(sum(A,2)==3);
    for l=1:size(intensity{i,1},1)
        int(intensity{i,3}(l,1),1) = intensity{i,1}(l,1);
    end
    
%     figure
%     imshow(IM)
%     hold on
%     scatter(x,y,2);
%     clc   
end
points=[M [0;int]];
file_name=[file_path sprintf('%04d',m) '.mat'];
save(file_name,'points');
toc;
end 