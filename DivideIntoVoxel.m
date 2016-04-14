scanName = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',365);
M = load(scanName);
pts = pointCloud(M.SCAN.XYZ');
pts = pcdenoise(pts);
pts1=pts.Location;
pts.Normal=pcnormals(pts);
%[pts.Normal,~]= findPointNormals(pts1);
%%
%[xyz,nxyz,gr,ngr,pr,Nr,Vr]=ransac_plane(pts1,pts.Normal,pts.Count,4);
%%
[ptcloud,nptcloud,~]=groundPlane(pts,4);
%%
   figure
   pcshow(nptcloud);
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
%    figure
%    pcshow(pt);
%    set(gca,'color','black');
%    hold off
%    grid on;
%    alpha(0.3);
%    xlabel('X');
%    ylabel('Y');
%    zlabel('Z');
   figure
   pcshow(nptcloud);
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   figure
   pcshow(pts);
   set(gca,'color','black');
   hold off
   grid on;
   alpha(0.3);
   xlabel('X');
   ylabel('Y');
   zlabel('Z');
   plot_normal(nptcloud.Location,nptcloud.Normal,3);
%%
       OT=OcTree(nptcloud.Location,'binCapacity',3800,'style','weighted');
       OT.shrink
 figure 
       boxH = OT.plot; 
       cols = lines(OT.BinCount); 
       doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:}); 
       for i = 1:OT.BinCount 
           set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OT.BinDepths(i)) 
           doplot3(pts(OT.PointBins==i,:),'.','Color',cols(i,:)) 
       end 
       axis image, view(3)
       hold on
%%