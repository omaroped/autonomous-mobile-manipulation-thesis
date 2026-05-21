#!/bin/bash
echo "Cleaning simulation state..."
pkill -9 -f gazebo
pkill -9 -f gz
pkill -9 -f ros
pkill -9 -f rviz
sleep 1
rm -rf ~/.ros/log/*
rm -rf ~/.gazebo/log/*
echo "Done! You can now restart the simulation."
