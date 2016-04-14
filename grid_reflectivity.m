function [ScanImage,ScanBlur]=grid_reflectivity(npts,reflectivity,xlimit,ylimit)
 x_min= -ceil(xlimit/2);
 y_min= -ceil(ylimit/2);
no_of_points=length(reflectivity);
x_size=ceil(xlimit/2);
y_size=ceil(ylimit/2);
% x_size=ceil(xlimit*2);
% y_size=ceil(ylimit*2);
e=zeros(x_size,y_size);
grid=cell(x_size,y_size);

for i=1:no_of_points
    x_loc=ceil((npts(i,1)-x_min)/0.5-x_size*1.75);
    y_loc=ceil((npts(i,2)-y_min)/0.5-y_size*1.5);
       if (x_loc<=x_size && x_loc>0) && (y_loc<=y_size && y_loc>0)
     grid{x_loc,y_loc}=[grid{x_loc,y_loc};reflectivity(i)];
       end
end
for j=1:size(grid,1)
    for k=1:size(grid,2)
     A=grid{j,k};
     if size(A,1)>=3
     e(j,k) = var(A);
     else
     e(j,k)=0;
     end 
    end
end

 e_max=max(max(e));
 ScanImage=e/e_max;


 ScanBlur = imgaussfilt(ScanImage,1);
ScanBlur = (ScanBlur - min(min(ScanBlur)))./((max(max(ScanBlur)) - min(min(ScanBlur))));