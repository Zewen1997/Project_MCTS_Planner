

pybind11_add_module(MCTS 
   src/main.cpp 
   src/mcts_planner.cpp 
   src/mcts_state.cpp
   src/mcts_timer.cpp
   src/mcts_data_handler.cpp
   src/mcts_exp3.cpp)

target_compile_definitions(MCTS PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

 