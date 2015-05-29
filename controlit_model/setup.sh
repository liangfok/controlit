# add each model directory to ROS_PACKAGE_PATH because the 'manifest.xml' file in the models directory is gazebo
# related and screws up ROS tools

# Get package's absolute path
rapid_model="$(rospack find rapid_model)"

# nasa_models=
for d in `find $rapid_model/models -mindepth 1 -maxdepth 1 -type d`
do
	# nasa_models=$d:$nasa_models;
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$d
done

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$rapid_model/models