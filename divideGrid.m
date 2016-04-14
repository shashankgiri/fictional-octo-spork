scanName = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',200);
M1 = load(scanName);
points1=M1.SCAN.XYZ';
[ScanImage1,ScanBlur1]=grid2D(points1);
scanName = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',202);
M2 = load(scanName);
points2=M2.SCAN.XYZ';
[ScanImage2,ScanBlur2]=grid2D(points2);
%%
H=hist(ScanBlur1,ScanBlur2,256);
mi=mutual_information(H);
%%
% figure(1)
% subplot(2,1,1)
% title('Normal Scan Image of non-ground points of scan 1')
% imagesc(ScanImage1);
% subplot(2,1,2); 
% title('Gaussian blured Scan Image of non-ground points of scan 1')
% imagesc(ScanBlur1);
% figure(2)
% subplot(2,1,1);
% title('Normal Scan Image of non-ground points of scan 2')
% imagesc(ScanImage2);
% subplot(2,1,2)
% title('Gaussian blured Scan Image of non-ground points of scan 2')
% imagesc(ScanBlur2);
% figure(3)
% bar(H,'DisplayName','H')