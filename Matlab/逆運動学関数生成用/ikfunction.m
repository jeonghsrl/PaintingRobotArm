% リンクパラメータ
L1 = 0.270;         %link 3の長さ
L2 = 0.13;          %L2長さのスタート、終了長さ
L3 = 0.18;          %0.20&L6:0.13, 0.18&L60.15 %L3長さのスタート、終了長さ
L4 = 0;             %link 4の長さ
L5 = 0;             %link 5の長さ
L6 = 0.15;          %link 6の長さ（メカニカルポイントから手先先端原点までの距離）

% シミュレーション条件追加
%robot bodytree生成
robot = func_sixLinkModel(L1,L2,L3,L4,L5,L6);                
%関節角度制限
robot.Bodies{1,1}.Joint.PositionLimits=[-179*3.14/180 179*3.14/180];
robot.Bodies{1,2}.Joint.PositionLimits=[-30*3.14/180 150*3.14/180];
robot.Bodies{1,3}.Joint.PositionLimits=[-90*3.14/180 90*3.14/180];
robot.Bodies{1,4}.Joint.PositionLimits=[  -45*3.14/180 315*3.14/180];
robot.Bodies{1,5}.Joint.PositionLimits=[ -150*3.14/180 0*3.14/180];  
robot.Bodies{1,6}.Joint.PositionLimits=[  -90*3.14/180 270*3.14/180];
%重力加速度
%robot.Gravity=[0,0,-9.8];    


% analyticalInverseKinematics方法。GenerateIKFunctionでmfileが生成されるので、それを使う。
aik = analyticalInverseKinematics(robot);
showdetails(aik)
generateIKFunction(aik,'robotIK');
%qInitial = q0;
%ikConfig = robotIK(toolForm(:,:,1))
