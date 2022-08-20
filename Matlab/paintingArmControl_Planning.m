%URDFからロボットモデルデータの作成(１回のみ)
if(0)
    smimport("paintingArm_v2021_v0_1.urdf")
    [paintingArm, importInfo] = importrobot(gcs);
    paintingArm.DataFormat = 'column';
    save paintingArm    %paintingArm.matが生成されるので、matlabではこれをロードする。
end

%URDFから鉄柱のモデルデータの作成(１回のみ)
if(0)
    smimport("pillar30_URDF_stl.urdf")
end


%%
clear
%URDFから生成されたモデルデータを利用する場合 paintingArm.matを読み込む
if(0)
    load paintingArm.mat
end

%% 軌道生成のため鉄柱モデルデータをurdfから読み込み構造体を作る。
ironPillar = importrobot('pillar30_URDF_stl.urdf');
ironPillarconfig = homeConfiguration(ironPillar);
%show(ironPillar,config,'Collisions','on','Visuals','on'); %configに従ったrobotを表示

%% 鉄柱L鋼の平面、点モデル
%---番目鋼材（斜め鋼材50x50x6）
%S1...S10は、10平面に属する点の姿勢と位置の同時変換行列, pillarObjPointsは、点の位置ベクトル
Lx1=0.044; Ly1=0.044; Lz1=0.430; Lt1=0.006;   %端点生成用

%構造物のL鋼1への同時変換行列計算
tformModel= trvec2tform([0.210 0 0.265]);
tformModel= tformModel*axang2tform([1 0 0 pi/3]);
tformModel= tformModel*axang2tform([0 0 1 pi]);

[S11,S12,S13,S14,S15,S16,S17,S18,S19,S110,obj1CenPoints,obj1Points]=func_pillarModelPoints(Lx1,Ly1,Lz1,Lt1,tformModel);

%plot3(obj1Points(:,1),obj1Points(:,2),obj1Points(:,3),'*');
%plot3(obj1CenPoints(:,1),obj1CenPoints(:,2),obj1CenPoints(:,3),'*');

%---２番目鋼材（左側75x75x9）
%S1...S10は、10平面に属する点の姿勢と位置の同時変換行列, pillarObjPointsは、点の位置ベクトル
Lx2=0.066; Ly2=0.066; Lz2=0.269; Lt2=0.009;
%構造物のL鋼1への同時変換行列計算
tformModel= trvec2tform([0.216 0.183 0.265]);
tformModel= tformModel*axang2tform([0 0 1 pi]);

[S21,S22,S23,S24,S25,S26,S27,S28,S29,S210,obj2CenPoints,obj2Points]=func_pillarModelPoints(Lx2,Ly2,Lz2,Lt2,tformModel);

%plot3(obj2Points(:,1),obj2Points(:,2),obj2Points(:,3),'*');
%plot3(obj2CenPoints(:,1),obj2CenPoints(:,2),obj2CenPoints(:,3),'*');

%----- 3番目の鋼材(右側75x75x9)
%S1...S10は、10平面に属する点の姿勢と位置の同時変換行列, pillarObjPointsは、点の位置ベクトル
Lx2=0.066; Ly2=0.066; Lz2=0.269; Lt2=0.009;
%構造物のL鋼1への同時変換行列計算
tformModel= trvec2tform([0.216 -0.183 0.265]);
tformModel= tformModel*axang2tform([0 1 0 pi]);

[S31,S32,S33,S34,S35,S36,S37,S38,S39,S310,obj3CenPoints,obj3Points]=func_pillarModelPoints(Lx2,Ly2,Lz2,Lt2,tformModel);

%plot3(obj3Points(:,1),obj3Points(:,2),obj3Points(:,3),'*')
%plot3(obj3CenPoints(:,1),obj3CenPoints(:,2),obj3CenPoints(:,3),'*');

%% 
% リンクパラメータ
L1 = 0.270;         %link 3の長さ
L2 = 0.13;          %L2長さのスタート、終了長さ
L3 = 0.18;          %0.20&L6:0.13, 0.18&L60.15 %L3長さのスタート、終了長さ
L4 = 0;             %link 4の長さ
L5 = 0;             %link 5の長さ
L6 = 0.15;          %link 6の長さ（メカニカルポイントから手先先端原点までの距離）

% シミュレーション条件追加
%robot bodytree生成
robot = func_sixLinkCollisionModel(L1,L2,L3,L4,L5,L6);                
%関節角度制限
robot.Bodies{1,1}.Joint.PositionLimits=[-179*3.14/180 179*3.14/180];
robot.Bodies{1,2}.Joint.PositionLimits=[-30*3.14/180 150*3.14/180];
robot.Bodies{1,3}.Joint.PositionLimits=[-90*3.14/180 90*3.14/180];
robot.Bodies{1,4}.Joint.PositionLimits= [-300*3.14/180 360*3.14/180]; % [-45*3.14/180 315*3.14/180];   %[-180*3.14/180 315*3.14/180]  
robot.Bodies{1,5}.Joint.PositionLimits=[ -150*3.14/180 0*3.14/180];  
robot.Bodies{1,6}.Joint.PositionLimits=[ -270*3.14/180 270*3.14/180];    % -90*3.14/180 270*3.14/180
%重力加速度
robot.Gravity=[0,0,-9.8];    
%% 
% 逆運動学ソルバーの準備
%            figure
%q0 = homeConfiguration(robot);        %初期configuration
q0 = [0;140*3.14/180; -40*3.14/180; 0; -90*3.14/180;0];
%q0 = [0;0;0; 0;0;0];
show(robot,q0,'Collisions','on','Visuals','on');      %最初のコンフィギュレーション
ndof = length(q0);                    %自由度の大きさ
qs = zeros(1, ndof);                  %countごとに、2dofのconfを返す行列を割り当てる

% 逆運動学ソルバー作成
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.SolutionTolerance = 0.01;  %成功判定誤差閾値
ik.SolverParameters.MaxIterations=1500;
weights = [0.1, 0.1, 0.1, 1, 1, 1];       %姿勢と位置の重み　姿勢角0.01rad
endEffector = 'tool';
qInitial = q0;

%初期toolのポーズ
InitialPose = getTransform(robot,qInitial,'tool','base')   

% analyticalInverseKinematics方法。GenerateIKFunctionでmfileが生成されるので、それを使う。
%ik = analyticalInverseKinematics(robot);
%showdetails(ik)
%generateIKFunction(ik,'robotIK');
%qInitial = q0;
%ikConfig = robotIK(toolForm(:,:,1))

%% 1方面の塗装目標点のポーズ
P_RoI(:,:,1)=S11(:,:,3)*trvec2tform([0 0.044/2 0.05]);             %ok
P_RoI(:,:,2)=S11(:,:,4)*trvec2tform([0 0.044/2. -0.05]);           %ok
P_RoI(:,:,3)=(S12(:,:,1)*trvec2tform([0 -0.044/2+0.006 0.09]))
P_RoI(:,:,4)=(S12(:,:,2)*trvec2tform([0 -0.044/2+0.006 -0.10]))
P_RoI(:,:,5)=(S14(:,:,3)*trvec2tform([0 0.044/2+0.006 0.05]))
P_RoI(:,:,6)=S14(:,:,4)*trvec2tform([0 0.044/2+0.006 -0.05]);
P_RoI(:,:,7)=S21(:,:,3)*trvec2tform([0 Ly2/2 0.09]); 
P_RoI(:,:,8)=S21(:,:,4)*trvec2tform([0 Ly2/2 -0.03]);
P_RoI(:,:,9)=S31(:,:,3)*trvec2tform([0 Ly2/2 0.05]);
P_RoI(:,:,10)=S31(:,:,4)*trvec2tform([0 Ly2/2 -0.05]);
P_RoI(:,:,11)=S11(:,:,3)*trvec2tform([0 0 Lz1/2]);    %正面中間点

%% 平面座標系を手先座標系に変換する関数を用意して、平面座標系ですべて設定する。

filename='setPaintingTrajePoints.dat';
%-----------------------------------
writematrix('#include <DynamixelWorkbench.h>',filename);
writematrix('#include "setup_para.h"',filename,'WriteMode','append');
writematrix('#include "dynamixelHsrlLib.h"',filename,'WriteMode','append');
writematrix('extern DYNAMIXEL_JOINT DJ[];',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');

%% 第一平面
i=1;
motionNum = 1;
qs=qInitial;
for taskCount=1:8
switch motionNum
    case 1   %①点の手前に刷毛を30度傾けた姿勢で移動 
        EEPose_S = InitialPose;   %現在ポーズからxyz移動
        EEPose_F = trvec2tform([-0.05 0 0])*convertEEtf(P_RoI(:,:,1)*axang2tform([0 1 0 -pi/6]));  %刷毛を30度傾ける                                      
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=2;
    case 2   %①点に刷毛をつける
        EEPose_S = EEPose_F;
        EEPose_F = convertEEtf(P_RoI(:,:,1)*axang2tform([0 1 0 -pi/6]));    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=3;
    case 3  %①点から②点へ刷毛を傾けたまま移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,2)*axang2tform([0 1 0 0]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=31;
    case 31 %②点到達後、刷毛を挙げて姿勢を逆方向に30度傾ける
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.05 0 0])*convertEEtf(P_RoI(:,:,2)*axang2tform([0 1 0 pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=32;
     case 32 %②点に刷毛をつける
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,2)*axang2tform([0 1 0 pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=4;       
    case 4   %②点から①点へ刷毛を傾けたまま移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,1)*axang2tform([0 1 0 0]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=5;
    case 5  %①点到達後刷毛をまっすに挙げる
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.05 0 0])*convertEEtf(P_RoI(:,:,1));    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=6;
    case 6  %初期姿勢へ戻る
        EEPose_S = EEPose_F;
        EEPose_F = InitialPose;    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
    otherwise
        disp('3')
end
 
% 逆運動学計算
for wayNum = 1:1:length(trajTimes)   
    [qSol solInfo] = ik(endEffector,tfInterp(:,:,wayNum),weights,qs); % 逆運動学計算
     qs = qSol;        %関節configuration解   
    %表示のために保存
    R_Config(i,:)=[taskCount wayNum qs'];  %
    R_cnt(taskCount)=length(trajTimes);
    i=i+1;
end
end
%%
JointTraj = getJointTrajectory(R_Config)
outputTrajePoints(filename,R_Config,'uint8_t setTrajePoints_S1(){ ')
%% 第2平面
i=1;
motionNum = 1;
qs=qInitial;
for taskCount=1:10
switch motionNum
    case 1 %②位置までまず移動。
        EEPose_S = InitialPose;   %現在ポーズからxyz移動
        EEPose_F = trvec2tform([-0.03 0.02 -0.02])*convertEEtf(P_RoI(:,:,2));                                        
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=11;
    case 11 %④位置手前の位置に移動
        EEPose_S =  EEPose_F;
        EEPose_F = trvec2tform([0 0 -0.02])*convertEEtf(P_RoI(:,:,4)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/6]));                                        
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=2;
    case 2  %④に接触
        EEPose_S = EEPose_F;
        EEPose_F = convertEEtf(P_RoI(:,:,4)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/6]));           
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=3;
    case 3  %③へ移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,3)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=31;
    case 31  %③から離れる
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.02 0 -0.02])*convertEEtf(P_RoI(:,:,3)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/3]));       
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=32;
     case 32  %③に接触
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,3)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=4;       
    case 4 %④に移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,4)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=5;
    case 5 %④から離れる
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([0 0 -0.02])*convertEEtf(P_RoI(:,:,4)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 -pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=51;
    case 51 %②の位置に移動
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.03 0.02 -0.02])*convertEEtf(P_RoI(:,:,2));      
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=6;
    case 6 %初期ポーズに移動
        EEPose_S = EEPose_F;
        EEPose_F = InitialPose;    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
    otherwise
        disp('3')
end

% 逆運動学計算
for wayNum = 1:1:length(trajTimes)   
    [qSol solInfo] = ik(endEffector,tfInterp(:,:,wayNum),weights,qs); % 逆運動学計算
     qs = qSol;        %関節configuration解   
    %表示のために保存
    R_Config(i,:)=[taskCount wayNum qs'];  %
    R_cnt(taskCount)=length(trajTimes);
    i=i+1;
end
end
%%
JointTraj = getJointTrajectory(R_Config)
outputTrajePoints(filename,R_Config,'uint8_t setTrajePoints_S2(){ ')
%% 第3平面
i=1;
motionNum = 1;
qs=qInitial;
for taskCount=1:8
switch motionNum
    case 1   %⑤位置の手前に移動
        EEPose_S = InitialPose;   
        EEPose_F = trvec2tform([-0.03 0 0.05])*convertEEtf(P_RoI(:,:,5)*axang2tform([0 1 0 -pi/3])*axang2tform([0 0 1 pi/10]));                                        
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=2;
    case 2   %⑤位置に接触              
        EEPose_S = EEPose_F;
        EEPose_F = convertEEtf(P_RoI(:,:,5)*axang2tform([0 1 0 -pi/4])*axang2tform([0 0 1 pi/10]));   
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=3;
    case 3 %⑥位置に移動    
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,6)*axang2tform([0 1 0 -pi/6])*axang2tform([0 0 1 pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=31;
    case 31 %⑥位置から離しながら姿勢変更  
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([0 0 0.03])*convertEEtf(P_RoI(:,:,6)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 pi/6]));       
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=32;
     case 32 %⑥位置に接触 
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,6)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=4;       
    case 4 %⑤位置に移動 
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,5)*axang2tform([0 1 0 -pi/4])*axang2tform([0 0 1 pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=5;
    case 5 %⑤位置からはなす 
        EEPose_S = EEPose_F;
        EEPose_F =  trvec2tform([-0.03 0 0.05])*convertEEtf(P_RoI(:,:,5)*axang2tform([0 1 0 -pi/3])*axang2tform([0 0 1 pi/10]));      
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=6;
    case 6 %Initial poseへ戻る 
        EEPose_S = EEPose_F;
        EEPose_F = InitialPose;    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
    otherwise
        disp('3')
end


% 逆運動学計算
for wayNum = 1:1:length(trajTimes)   
    [qSol solInfo] = ik(endEffector,tfInterp(:,:,wayNum),weights,qs); % 逆運動学計算
     qs = qSol;        %関節configuration解   
    %表示のために保存
    R_Config(i,:)=[taskCount wayNum qs'];  %
    R_cnt(taskCount)=length(trajTimes);
    i=i+1;
end
end

%%
JointTraj = getJointTrajectory(R_Config)
outputTrajePoints(filename,R_Config,'uint8_t setTrajePoints_S3(){ ')
%% 第4平面
i=1;
motionNum = 1;
qs=qInitial;
for taskCount=1:8
switch motionNum
    case 1   %InitialPoseから⑦の手前位置へ。接触しないようにZ軸方向へ傾ける
        EEPose_S = InitialPose;   %現在ポーズからxyz移動
        EEPose_F = trvec2tform([-0.06 0 0 ])*convertEEtf(P_RoI(:,:,7)*axang2tform([0 1 0 -pi/6])*axang2tform([0 0 1 pi/6]))                                        
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=2;
    case 2   %⑦の位置へ接触
        EEPose_S = EEPose_F;   
        EEPose_F = convertEEtf(P_RoI(:,:,7)*axang2tform([0 1 0 -pi/6])*axang2tform([0 0 1 pi/6]));    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=3;
    case 3   %⑧の位置へ移動。接触しないようにZ軸周りをすこしあげる
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,8)*axang2tform([0 0 1 pi/4]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=31;
    case 31  %⑧位置から少しあげながら、姿勢を変換。
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.01 0 0])*convertEEtf(P_RoI(:,:,8)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 pi/4]));       
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=32;
     case 32 %⑧位置につける。
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,8)*axang2tform([0 1 0 pi/6])*axang2tform([0 0 1 pi/4]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=4;       
    case 4  %⑦位置へ移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,7)*axang2tform([0 1 0 -pi/10])*axang2tform([0 0 1 pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=5;
    case 5 %⑦から離す
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.06 0 0])*convertEEtf(P_RoI(:,:,7)*axang2tform([0 1 0 -pi/10])*axang2tform([0 0 1 pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=6;
    case 6 %initial poseへ戻す
        EEPose_S = EEPose_F;
        EEPose_F = InitialPose;    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
    otherwise
        disp('3')
end

% 逆運動学計算

% 逆運動学計算
for wayNum = 1:1:length(trajTimes)   
    [qSol solInfo] = ik(endEffector,tfInterp(:,:,wayNum),weights,qs); % 逆運動学計算
     qs = qSol;        %関節configuration解   
    %表示のために保存
    R_Config(i,:)=[taskCount wayNum qs'];  %
    R_cnt(taskCount)=length(trajTimes);
    i=i+1;
end

end
%%
JointTraj = getJointTrajectory(R_Config)
outputTrajePoints(filename,R_Config,'uint8_t setTrajePoints_S4(){ ')
%% 第5平面
i=1;
motionNum = 1;
qs=qInitial;
for taskCount=1:8
switch motionNum
    case 1 %InitialPoseから⑨手前位置へ。接触しないようにZ軸方向へ傾ける
        EEPose_S = InitialPose;   %現在ポーズからxyz移動
        EEPose_F = trvec2tform([-0.06 0 0])*convertEEtf(P_RoI(:,:,9)*axang2tform([0 1 0 -pi/6])*axang2tform([0 0 1 pi/6]));                                        
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=2;
    case 2 %⑨位置に接触
        EEPose_S = EEPose_F;
        EEPose_F = convertEEtf(P_RoI(:,:,9)*axang2tform([0 1 0 -pi/6])*axang2tform([0 0 1 pi/6]));    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=3;
    case 3 %⑩位置へ移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,10)*axang2tform([0 1 0 pi/3])*axang2tform([0 0 1 pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=31;
    case 31 %⑩位置から離しながら姿勢変更
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.01 0 0])*convertEEtf(P_RoI(:,:,10)*axang2tform([0 1 0 pi/3])*axang2tform([0 0 1 pi/6]));       
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=32;
     case 32 %⑩位置につける
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,10)*axang2tform([0 1 0 pi/3])*axang2tform([0 0 1 pi/6]));     
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=4;       
    case 4 %⑨位置へ移動
        EEPose_S = EEPose_F;
        EEPose_F =convertEEtf(P_RoI(:,:,9)*axang2tform([0 1 0 0])*axang2tform([0 0 1 pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=5;
    case 5  %⑨位置から離す
        EEPose_S = EEPose_F;
        EEPose_F = trvec2tform([-0.06 0 0])*convertEEtf(P_RoI(:,:,9)*axang2tform([0 1 0 0])*axang2tform([0 0 1 pi/6]))    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
        motionNum=6;
    case 6  %initialposeへ戻る
        EEPose_S = EEPose_F;
        EEPose_F = InitialPose;    
        [tfInterp, v1,trajTimes] = makeTraj(EEPose_S, EEPose_F);
    otherwise
        disp('3')
end
 
% 逆運動学計算
for wayNum = 1:1:length(trajTimes)   
    [qSol solInfo] = ik(endEffector,tfInterp(:,:,wayNum),weights,qs); % 逆運動学計算
     qs = qSol;        %関節configuration解   
    %表示のために保存
    R_Config(i,:)=[taskCount wayNum qs'];  %
    R_cnt(taskCount)=length(trajTimes);
    i=i+1;
end
end
%%
JointTraj = getJointTrajectory(R_Config)
outputTrajePoints(filename,R_Config,'uint8_t setTrajePoints_S5(){ ')
%% 塗装動作制御用simulink blockを立ち上げる
open_system('paintingControl.slx');

%% 軌道生成
function [tfInterp, v1,trajTimes] = makeTraj(tformS, tformF)

timeStep = 0.01;   %seconds
toolSpeed = 0.2;  %m/s

taskInit = tformS;
taskFinal = tformF;

%２ポーズ間の移動距離計算
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));
%移動距離とツール速度に基づき、軌跡の時間を定義
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1) ; trajTimes(end)];

[tfInterp, v1] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes);

end
%%  convert to EE pose
function EE_tf = convertEEtf(ROI_tform)
    ROI_tform = ROI_tform*axang2tform([0 1 0 -pi/2]);      %刷毛姿勢への変換(目標姿勢)
    EE_tf =  ROI_tform*axang2tform([0 0 1 pi]);
end

%% % rosテスト用
function outputTrajePoints(filename,R_Config,strfunc)

trajPoints(1,:)= round(R_Config(:,3)'*180/3.14,2);
trajPoints(2,:)= round(R_Config(:,4)'*180/3.14,2);
trajPoints(3,:)= round(R_Config(:,5)'*180/3.14,2);
trajPoints(4,:)= round(R_Config(:,6)'*180/3.14,2)
trajPoints(5,:)= round(R_Config(:,7)'*180/3.14,2);
trajPoints(6,:)= round(R_Config(:,8)'*180/3.14,2);


writematrix(strfunc,filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');

str1 = 'uint32_t tSize = ';  str2 = string(length(trajPoints));  strSize = append(str1,str2,';');
writematrix(strSize,filename,'WriteMode','append');

writematrix('',filename,'WriteMode','append');

writematrix('float trpointsJ1[tSize]={',filename,'WriteMode','append');
writematrix(trajPoints(1,:),filename,'Delimiter',',','WriteMode','append');
writematrix('};',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');
writematrix('float trpointsJ2[tSize]={',filename,'WriteMode','append');
writematrix(trajPoints(2,:),filename,'Delimiter',',','WriteMode','append');
writematrix('};',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');
writematrix('float trpointsJ3[tSize]={',filename,'WriteMode','append');
writematrix(trajPoints(3,:),filename,'Delimiter',',','WriteMode','append');
writematrix('};',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');
writematrix('float trpointsJ4[tSize]={',filename,'WriteMode','append');
writematrix(trajPoints(4,:),filename,'Delimiter',',','WriteMode','append');
writematrix('};',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');
writematrix('float trpointsJ5[tSize]={',filename,'WriteMode','append');
writematrix(trajPoints(5,:),filename,'Delimiter',',','WriteMode','append');
writematrix('};',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');
writematrix('float trpointsJ6[tSize]={',filename,'WriteMode','append');
writematrix(trajPoints(6,:),filename,'Delimiter',',','WriteMode','append');
writematrix('};',filename,'WriteMode','append');
writematrix('',filename,'WriteMode','append');

str={'for(uint16_t i=0;i<tSize;i++){'; 'DJ[J1].trajPoints[i]=trpointsJ1[i];'; 
     'DJ[J2].trajPoints[i]=trpointsJ2[i];';
     'DJ[J3].trajPoints[i]=trpointsJ3[i];'; 
     'DJ[J4].trajPoints[i]=trpointsJ4[i];';
     'DJ[J5].trajPoints[i]=trpointsJ5[i];'; 
     'DJ[J6].trajPoints[i]=trpointsJ6[i];'; 
     '}';
     '';
     'for(uint8_t i=Jnum_S; i <= Jnum_F ;i++) DJ[i].trIdxMax =tSize;';}

for i=1:1:10
writecell(str(i),filename,'WriteMode','append');
end
writematrix('',filename,'WriteMode','append');
writematrix('}',filename,'WriteMode','append');
%--------------------------
end
%%
