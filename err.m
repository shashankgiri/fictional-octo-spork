function [mt,mr]=err(pose,t,a)
TT = pose(1:3,1);
TR = pose(4:6,1);
mt = (TT-t).^2;
mr = (TR-a).^2;