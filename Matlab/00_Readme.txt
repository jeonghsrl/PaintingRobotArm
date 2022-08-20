
paintingArmControl.m: pillarのtargetposeの軌道制御
paintingControl.slx: 6自由度アームの軌道追従制御ブロック。paintingArmControl.mから呼び出される。
sixLinkModelAnalysis_linkParaFixed.m: 塗装アーム試作機3dprintのリンクパラメータ微調整用プログラム
                                      リンク長さを調整して、pillarの各ターゲットポーズが実現できるのかシミュレーション

func_pillarModelPoints.m : 鉄柱モデルの角のポイントを返す関数
func_sixLinkCollisionModel.m: 6自由度アームrigidモデル構造体を返す関数
paintingArm.mat: "paintingArm_v2021_v0_1.urdf"をsimulinkにimportして作成したロボットモデルの保存データ
                  urdfを使ったロボットモデルを使う場合は、これをloadする。