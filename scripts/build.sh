#!/bin/sh

NAME=$1

if [ -z "$1" ]; then
    echo "enter app name"
    exit 1
fi

mkdir src
cd src
rm -rf ${NAME}_gazebo
rm -rf ${NAME}_description
rm -rf ${NAME}_bringup

ros2 pkg create ${NAME}_gazebo --build-type ament_cmake 
ros2 pkg create ${NAME}_description --build-type ament_cmake 
ros2 pkg create ${NAME}_bringup --build-type ament_cmake

cd -    
sed -i "s/XXX/$NAME/g" .vscode/tasks.json
sed -i "s/XXX/$NAME/g" .devcontainer/devcontainer.json

mkdir src/${NAME}_gazebo/env-hooks
cat <<EOF >>src/${NAME}_gazebo/env-hooks/${NAME}_gazebo.dsv.in
prepend-non-duplicate;GAZEBO_RESOURCE_PATH;share/@PROJECT_NAME@/worlds
prepend-non-duplicate;GAZEBO_PLUGIN_PATH;share/@PROJECT_NAME@/bin
EOF


mkdir src/${NAME}_description/env-hooks
cat <<EOF >>src/${NAME}_description/env-hooks/${NAME}_description.dsv.in
prepend-non-duplicate;GAZEBO_MODEL_PATH;share/@PROJECT_NAME@/models
prepend-non-duplicate;GAZEBO_RESOURCE_PATH;share/@PROJECT_NAME@/meshes
EOF

sed -i "\$i ament_environment_hooks(\"\${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/\${PROJECT_NAME}.dsv.in\")" src/${NAME}_gazebo/CMakeLists.txt
sed -i "\$i ament_environment_hooks(\"\${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/\${PROJECT_NAME}.dsv.in\")" src/${NAME}_description/CMakeLists.txt