# Autoware_type_changer

## Install
1. Autowareのinstallをする
<https://github.com/MeijoMeguroLab/AutowareArchitectureProposal.proj>

2. ワークスペースを作成し，autoware_type_changerをInstallする
```
mkdir -p ~/iv_ws/src
cd ~/iv_ws/src
git clone git@github.com:YutaHoda/autoware_type_changer.git
cd ~/iv_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## How to Use

- Autoware.IV
	shのmap_path,(sensor_model，vehicle_model)を変更する．
		map_pathは/map/pcd/以下にpcdファイル（複数可能）を置くようにする

	autoware_type_changer，navsatfix2poseをクローンしたワークスペースがiv_wsではない場合，shのbashを通しているワークスペース名を変更する．

- autoware_type_changer
	launchファイルのsub_topic_nameを確認する．
		*デフォルト*
		LiDAR：/velodyne_packets
		車輪速：/can_twist
		IMU：/imu/data_raw

- navsatfix2pose
	launchファイルのsub_gnss_topic_nameを確認する
		*デフォルト*
		GNSS：/fix
	setup_gnssposetf.launchでTFを設定する
1. ターミナル1
```
cd AutowareArchitectureProposal.proj/
shでlaunchファイルを起動
```

2. ターミナル2
```
rosbag play --clock <rosbag file>
```