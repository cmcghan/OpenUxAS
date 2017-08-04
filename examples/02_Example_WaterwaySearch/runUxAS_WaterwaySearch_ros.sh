#! /bin/bash

SAVE_DIR=$(pwd)

RM_DATAWORK="rm -R ./datawork"
RM_LOG="rm -R ./log"

BIN="../../../build/uxas"

#mkdir -p RUNDIR_WaterwaySearch # OLD
#cd RUNDIR_WaterwaySearch # OLD
mkdir -p RUNDIR_WaterwaySearch_ros # NEW
cd RUNDIR_WaterwaySearch_ros # NEW
$RM_DATAWORK
$RM_LOG
#$BIN -cfgPath ../cfg_WaterwaySearch.xml # OLD
$BIN -cfgPath ../cfg_WaterwaySearch_ros.xml # NEW

