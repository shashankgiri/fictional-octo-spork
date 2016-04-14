clc
clearvars
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','F:\',200);
M1=load(scanName1);
pts1 = pointCloud(M1.SCAN.XYZ');
pts1.Normal=pcnormals(pts1);
[ptc1,nptc1,plane1,p1]=gp(pts1);
gridStep = 0.5;
nptc1 = pcdownsample(nptc1,'gridAverage',gridStep);
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','F:\',205);
M2=load(scanName2);
pts2 = pointCloud(M2.SCAN.XYZ');
pts2.Normal=pcnormals(pts2);
[ptc2,nptc2,plane2,p2]=gp(pts2);
nptc2 = pcdownsample(nptc2,'gridAverage',gridStep);
xlimit2=nptc2.XLimits(1,2)-nptc2.XLimits(1,1);
ylimit2=nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit=max(xlimit1,xlimit2);
ylimit=max(ylimit1,ylimit2);
[SI1,SB1]=grid2D(nptc1.Location,xlimit,ylimit);
[SI1R,SB1R]=grid2DR(nptc1.Location,xlimit,ylimit);
[R12,eul1]=find_rotation(plane1,plane2);
t=R12*mean(pts1.Location)'-mean(pts2.Location)';
% ObjectiveFunction1=@(tf)mi2(tf,SB1R,nptc2,xlimit,ylimit);
% [tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,0);
tf=0.1175;
x=0.3;
i=0;
I=zeros(60,12);
lx=zeros(60,1);
ly=zeros(12,1);
for y=0:0.01:3
    i=i+1;
    ly(i,1)=y;
    j=1;
%for x=0:0.05:0.6
    %j=j+1;
    %lx(j,1)=x;
  I(i,j) = MI_2(x,y,tf,SB1,nptc2,xlimit,ylimit);
end
%end
%%
% Th=mean(mean(I));
% for m=1:size(I,1)
%     for n=1:size(I,2)
%    if I(m,n)<Th
%        I(m,n)=0;
% %    else 
% %        I(m,n)=max(max(I));
%     end
%     end
% end
%%
figure
%hold all
plot(ly',I);
%plot(lx)
%%
II = imgaussfilt(I,1);
figure
surf(I);
figure
imagesc(I);
figure
surf(II)