#! /bin/bash

mkdir ../build && cd ../build

cmake ..

make

cp /home/zewen/Project_MCTS_Planner/build/mcts_planner/MCTS.cpython-38-x86_64-linux-gnu.so /home/zewen/Project_MCTS_Planner/test

cd /home/zewen/Project_MCTS_Planner/test

python3 test_MCTS_Planner.py
