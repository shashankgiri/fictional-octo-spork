function [Rt,eul]=rot(V1,V2)
V3=cross(V1,V2)/norm(cross(V1,V2));
V4=cross(V3,V1);
M1=[V1';V4';V3'];
cos=dot(V2,V1);
sin=dot(V2,V4);
M2=[cos sin 0;-sin cos 0;0 0 1];
Rt=(M2*M1)\M1;
eul=rotm2eul(Rt);
end