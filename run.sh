rm -rf output
mkdir output

if [[ -z "$RUN_SCRIPT_DO_NOT_CLEAR" ]]; then
    unset RUN_SCRIPT_DO_NOT_CLEAR
    clear
fi

source src/harpia_msgs/install/setup.bash
source install/setup.bash

echo "Running project:"
ros2 launch route_executor2 patrol_example_launch.py ${1:+mission_index:=$1}