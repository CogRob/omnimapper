##Takes data, toolbox location and output directory as input and creates pcds

TUM_TOOLBOX_DIR="/home/siddharth/kinect/tum_benchmark/rgbd_benchmark_tools/"

ASSOCIATE_SCRIPT=$TUM_TOOLBOX_DIR"/associate.py"
DATA_DIR=$1

#associate rgb.txt and depth.txt given in TUM data
echo -n "Associating $DATA_DIR/rgb.txt and $DATA_DIR/depth.txt..."
python $ASSOCIATE_SCRIPT $DATA_DIR/rgb.txt $DATA_DIR/depth.txt > $DATA_DIR/associate.txt
echo "OK"

#take rgb.txt, depth.txt and associate.txt and convert it into pcd cloud
echo "Converting rgb.txt and depth.txt to pcd files... "
./build/tum2pcd $DATA_DIR/associate.txt $DATA_DIR/pcds/ 
echo "OK"





