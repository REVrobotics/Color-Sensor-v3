#! /bin/bash

BUILD_YEAR=2020
BUILD_REPO_DIR=./build/repos
RELEASE_DIR=$BUILD_REPO_DIR/releases/com/revrobotics/frc
MOVE_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/maven/com/revrobotics/frc
VENDOR_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/vendordeps

echo "*** Moving maven directories ***"
cp -r $RELEASE_DIR/ColorSensorV3-cpp $MOVE_DIR
cp -r $RELEASE_DIR/ColorSensorV3-java $MOVE_DIR

echo "*** Moving vendor deps ***"
cp -r ./vendordeps/* $VENDOR_DIR
