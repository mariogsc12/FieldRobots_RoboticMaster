#!/bin/bash

COPPELIA_PATH="api/coppeliaSim.sh"
SCENE_PATH="Ejemplo/big_fields_robots.ttt"

if [[ "$1" == "--empty" ]]; then
    echo "Launching CoppeliaSim with empty scene..."
    bash $COPPELIA_PATH
else
    echo "Launching CoppeliaSim with scene $SCENE_PATH..."
    bash $COPPELIA_PATH $SCENE_PATH
fi