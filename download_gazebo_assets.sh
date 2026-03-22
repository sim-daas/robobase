#!/bin/bash
# Script to download standard Gazebo assets

echo "Setting up ~/.gazebo/models directory..."
mkdir -p ~/.gazebo/models
cd ~/.gazebo/models

if [ ! -d "gazebo_models" ]; then
    echo "Cloning the osrf/gazebo_models repository (~500MB)..."
    git clone https://github.com/osrf/gazebo_models.git
fi

echo "Extracting needed models..."
cp -r gazebo_models/nist_maze_wall_240 .
cp -r gazebo_models/asphalt_plane .
cp -r gazebo_models/fountain .
cp -r gazebo_models/person_walking .
cp -r gazebo_models/person_standing .

echo "Cleaning up..."
rm -rf gazebo_models

echo "Assets have been installed in ~/.gazebo/models!"
