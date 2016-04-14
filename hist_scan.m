function H=hist_scan(im1,im2,hist_size)

X_size=size(im1,1);
Y_size=size(im1,2);
H=zeros(hist_size,hist_size);

x=floor(im1*(hist_size-1))+1;
y=floor(im2*(hist_size-1))+1;

for i=1:X_size
    for j=1:Y_size
        H(x(i,j),y(i,j))= H(x(i,j),y(i,j))+1;
    end
end
end