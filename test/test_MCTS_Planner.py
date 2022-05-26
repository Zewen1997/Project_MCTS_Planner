from argparse import Action
import MCTS

print("-------------------- test MCTS lib --------------------")

reference_path = [0, 0.5, 0.7]
waypoint1 = MCTS.Waypoint(0, 0, 0, 0.9, 0)
waypoint2 = MCTS.Waypoint(0.4, 0.3, 0.5, 0.88, 0)
waypoint3 = MCTS.Waypoint(0.5, 0.32, 0.7, 0.88, 0)
waypoint_ego = [waypoint1, waypoint2, waypoint3]




# MCTS Interface
mcts_interface = MCTS.MCTSInterface()
print(mcts_interface)

# MCTS EgoState
ego_state = MCTS.EgoState(0.16, 0.25, 0, 0.95, 5, 1.8, 5)
objects_state = []

# MCTS ObjectState
for i in range (10):
    objects_state.append(MCTS.ObjectState([2, 3], i+5, i+5, i+5, i+5, i+5, i+5, i+5))    

print(objects_state[1])


# MCTS StatesStatus
state_status = MCTS.StatesStatus(False, False)

# MCTS EnvironmentState 
mcts_state = MCTS.EnvironmentState(ego_state, objects_state,state_status)
mcts_state.print()

# set EnvironmentState with interface
mcts_interface.setEnvironmentState(mcts_state)

# MCTS MCTSPlanner 
mcts_planner = MCTS.MCTSPlanner()
print("done init planner", mcts_planner)

mcts_planner.interface().setEnvironmentState(mcts_state)

# MCTS process functions
mcts_planner.processInit()
mcts_planner.processRun()

# MCTS MCTSPlanner output through interface
decisions = mcts_planner.interface().getDecisions()
print("decisions ", decisions)

data_handler = MCTS.DataHandler()
print(data_handler.getNearstVehicles(ego_state, objects_state))