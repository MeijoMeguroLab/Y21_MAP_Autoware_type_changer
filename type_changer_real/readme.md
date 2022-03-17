自分のwsがある場合は、そこにいれてビルドする
ない場合
$ mkdir ~/type_change_ws/src
をして、src以下にtype_changerを置く

ビルド
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
$ rosrun type_changer type_change
