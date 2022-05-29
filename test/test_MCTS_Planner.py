from argparse import Action
import MCTS

print("-------------------- test MCTS lib --------------------")

reference_path = [0, 0.1835, 0.4]
waypoint1 = MCTS.Waypoint(0, 0, 0, 0.9, 0)
waypoint2 = MCTS.Waypoint(0.4, 0.3, 0.1835, 0.88, 0)
waypoint3 = MCTS.Waypoint(0.5, 0.32, 0.4, 0.88, 0)
waypoint_ego = [waypoint1, waypoint2, waypoint3]


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


# MCTS MCTSPlanner 
mcts_planner = MCTS.MCTSPlanner()
print("done init planner", mcts_planner)

# MCTS process functions
print("init:")
mcts_planner.processInit()
print("set state:")
mcts_planner.setDecisions([1.0,5.0,9.0])
print(mcts_planner.getDecisions())
# mcts_planner.processRun()

# Test update states funtion
mcts = MCTS.MCTSState()
mcts.createStartState(mcts_state)
mcts.setReferencePathEgo(reference_path)
mcts.setWaypointEgo(waypoint_ego)
action = MCTS.ActionType(2)
print(action)
next_state = mcts.calculateNextEgoState(mcts_state, action)
next_state.print()
