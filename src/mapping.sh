Y="\\033[33m"

root_dir="/home/ding/backup/code/haibo_lt_mapping"
child_dir=$(date +%y%m%d)
bag_name="test.bag"
lidar_topic="/lidar_aeb/raw_points"
imu_topic="/imu_topic"
# root_dir=$(pwd)
record_dir=$root_dir"/data/"$child_dir

if [ $# -eq 0 ]; then
echo -e "Usage: bash $0 [options] $Y"
echo "       record:   record ros bag and press ctrl-c to stop"
echo "       mapping:  build a global map, send out pointcloud and gird map"
exit -1
elif [ $# -gt 1 ]; then
echo "wrong number of arguments."
else
	case $1 in
		record)
		# 录图
		echo "recording ros bag..."
		if [ -d $record_dir ]; then
		   echo $record_dir"   exists"
		else
		   mkdir $record_dir
		fi
        cd $record_dir
		xterm -hold -e roscore &
		sleep 1
		echo "start recording..."
		rosbag record -O $bag_name $lidar_topic $imu_topic
		echo "recording completed"
		exit 1
		;;
		pointcloud)
		# 建点云图 
		source $root_dir/devel/setup.bash
		echo "start pointcloud map..."
		roslaunch rtk_mapping map.launch arg_bag_path:=$record_dir/$bag_name arg_map_path:=$record_dir
		echo "pointcloud map completed"
		exit 1
		;;
        mapping)
		# 建栅格图
		source $root_dir/devel/setup.bash
		echo "start mapping..."
		roslaunch global_ogm global_ogm.launch arg_bag_path:=$record_dir/$bag_name arg_map_path:=$record_dir
		echo " mapping completed"
		exit 1
		;;
		display)
		# 可视化
		source $root_dir/devel/setup.bash
		echo "start displaying..."
		roslaunch json_test json.launch arg_json_file_path:=$record_dir/global_map.json arg_point1_lat:=31.1377397 arg_point1_lon:=118.1788036 arg_pcd_file_path:=$record_dir/map.pcd
		echo "display completed"
		exit 1
		esac
		echo "wrong arguments."
		exit 0
fi



