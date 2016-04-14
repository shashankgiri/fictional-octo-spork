function [pose, tform] = load_pose(folder,ScanIndex1,ScanIndex2)
X_bl = [2.4;-0.01;-2.3;pi;0;pi/2];
imu = load_pose_applanix('Pose-Applanix.log');
scanName1 = sprintf('%s/SCANS/Scan%04d.mat',folder,ScanIndex1);
M1 = load(scanName1);
[i1,~,~,~] = findcind(imu.utime,M1.SCAN.timestamp_laser);
pose1 = [imu.pos(i1,:),imu.rph(i1,:)];
pose1 = ssc_head2tail(pose1',X_bl);
%%
scanName2 = sprintf('%s/SCANS/Scan%04d.mat',folder,ScanIndex2);
M2 = load(scanName2);
[i2,~,~,~] = findcind(imu.utime,M2.SCAN.timestamp_laser);
pose2 = [imu.pos(i2,:),imu.rph(i2,:)];
pose2 = ssc_head2tail(pose2',X_bl);
%%
pose_rel = ssc_tail2tail(pose1,pose2);
eul = [pose_rel(6,1) pose_rel(5,1) pose_rel(4,1)];
rt=eul2rotm(eul);
TT = [-pose_rel(1,1) pose_rel(2,1) -pose_rel(3,1)];
pose = [TT'; pose_rel(4:6,1)];
B=[rt,[0 0 0]';[TT 1]];
tform=affine3d(B);
