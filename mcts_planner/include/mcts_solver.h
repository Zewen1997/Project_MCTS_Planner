/**
 * @file    mcts_solver.h
 * @brief   Implementation of a Monte Carlo Tree Search (MCTS) Solver.
 */

#ifndef MCTS_SOLVER_H
#define MCTS_SOLVER_H

#include <cfloat>
#include "mcts_timer.h"
#include "mcts_tree_node.h"


template <class State, typename Action>
class MCTS
{
  typedef TreeNodeT<State, Action> TreeNode;

 private:
  LoopTimer timer_;
  int    iterations_;

  std::shared_ptr<TreeNode> root_node_;  // root node of tree

 public:
  float uct_k_;             // k value in UCT function. default = sqrt(2)
  int  max_iterations_;    // do a maximum of this many iterations (0 to run till end)
  int  max_millis_;        // run for a maximum of this many milliseconds (0 to run till end)
  int  simulation_depth_;  // how many ticks (frames) to run simulation for
  int  max_tree_depth_;    // maximum of tree depth

  //--------------------------------------------------------------
  MCTS() : iterations_(0), uct_k_(sqrt(2)), max_iterations_(100), max_millis_(0), simulation_depth_(10), max_tree_depth_(10) {}

  //--------------------------------------------------------------
  const LoopTimer& getTimer() const
  {
    return timer_;
  }

  const int getIterations() const
  {
    return iterations_;
  }

  //--------------------------------------------------------------
  // get best (immediate) child for given TreeNode based on uct score
  TreeNode* getBestUCTCchild(TreeNode* node, float uct_k) const
  {
    // sanity check
    if (!node->isFullyExpanded())
      return NULL;

    float   best_utc_score = -std::numeric_limits<float>::max();
    TreeNode* best_node      = NULL;

    // iterate all immediate children and find best UTC score
    int num_children = node->getNumChildren();
    for (int i = 0; i < num_children; i++)
    {
      TreeNode* child            = node->getChild(i);
      float   uct_exploitation = (float)child->getValue() / (child->getNumVisits() + FLT_EPSILON);
      float   uct_exploration  = sqrt(log((float)node->getNumVisits() + 1) / (child->getNumVisits() + FLT_EPSILON));
      float   uct_score        = uct_exploitation + uct_k * uct_exploration;

      if (uct_score > best_utc_score)
      {
        best_utc_score = uct_score;
        best_node      = child;
      }
    }

    return best_node;
  }

  //--------------------------------------------------------------
  TreeNode* getMostVisitedChild(TreeNode* node) const
  {
    int    most_visits = -1;
    TreeNode* best_node   = NULL;

    // iterate all immediate children and find most visited
    int num_children = node->getNumChildren();
    for (int i = 0; i < num_children; i++)
    {
      TreeNode* child = node->getChild(i);
      if (child->getNumVisits() > most_visits)
      {
        most_visits = child->getNumVisits();
        best_node   = child;
      }
    }

    return best_node;
  }

  //--------------------------------------------------------------
  std::vector<Action> getMostVisitedActions()
  {
    // Choose all most visited actions along the tree
    std::vector<Action> most_visited_actions;
    TreeNode*           choose_node = root_node_.get();

    while (choose_node != NULL && !choose_node->isTerminal() && choose_node->getNumChildren() > 0)
    {
      choose_node = getMostVisitedChild(choose_node);
      if (choose_node != NULL)
      {
        most_visited_actions.push_back(choose_node->getAction());
      }
    }
    return most_visited_actions;
  }

  //--------------------------------------------------------------
  std::vector<State> getMostVisitedStatesAlongTree()
  {
    // Choose all most visited states along the tree
    std::vector<State> most_visited_states;
    TreeNode*          choose_node = root_node_.get();

    while (choose_node != NULL && !choose_node->isTerminal() && choose_node->getNumChildren() > 0)
    {
      choose_node = getMostVisitedChild(choose_node);
      if (choose_node != NULL)
      {
        most_visited_states.push_back(choose_node->getState());
      }
    }
    return most_visited_states;
  }

  //--------------------------------------------------------------
  std::vector<Action> getBestUCTActions()
  {
    // Choose all best UCT actions along the tree
    std::vector<Action> best_uct_actions;
    TreeNode*           choose_node = root_node_.get();

    while (choose_node != NULL && !choose_node->is_terminal() && choose_node->get_num_children() > 0)
    {
      choose_node = get_best_uct_child(choose_node, uct_k_);
      if (choose_node != NULL)
      {
        best_uct_actions.push_back(choose_node->getAction());
      }
    }
    return best_uct_actions;
  }

  //--------------------------------------------------------------
  std::vector<State> getBestUCTStateAlongTree()
  {
    // Choose all best UCT actions along the tree
    std::vector<State> best_uct_states;
    TreeNode*          choose_node = root_node_.get();

    while (choose_node != NULL && !choose_node->is_terminal() && choose_node->get_num_children() > 0)
    {
      choose_node = get_best_uct_child(choose_node, uct_k_);
      if (choose_node != NULL)
      {
        best_uct_states.push_back(choose_node->getState());
      }
    }
    return best_uct_states;
  }

  //--------------------------------------------------------------
  void search(const State& current_state, int seed = 1, std::vector<State>* explored_states = nullptr)
  {
    // initialize timer
    timer_.init();

    // initialize root TreeNode with current state
    root_node_ = std::make_shared<TreeNode>(current_state);

    TreeNode* best_node = NULL;

    // iterate
    iterations_ = 0;
    while (true)
    {
      // indicate start of loop
      timer_.loopStart();

      // 1. SELECT. Start at root, dig down into tree using UCT on all fully expanded nodes
      TreeNode* node = root_node_.get();
      while (!node->isTerminal() && node->isFullyExpanded())
      {
        node = getBestUCTCchild(node, uct_k_);
        //	assert(node);	// sanity check
      }

      // 2. EXPAND by adding a single child (if not terminal or not fully expanded)
      if (!node->isFullyExpanded() && !node->isTerminal() && node->getDepth() <= max_tree_depth_)
        node = node->expand();

      State state(node->getState());

      // 3. SIMULATE (if not terminal)
      if (!node->isTerminal())
      {
        Action action;
        for (int t = 0; t < simulation_depth_; t++)
        {
          if (state.isTerminal())
            break;

          if (state.getRandomAction(action))
          {
            // current depth is node depth + simulation depth
            int current_depth = node->getDepth() + t;
            state.applyAction(action, current_depth);
          }
          else
            break;
        }
      }

      // get rewards vector for all agents
      const std::vector<float> rewards = state.evaluateRewards();

      // add to history
      if (explored_states)
        explored_states->push_back(state);

      // 4. BACK PROPAGATION
      while (node)
      {
        node->update(rewards);
        node = node->getParent();
      }

      // find most visited child
      best_node = getMostVisitedChild(root_node_.get());

      // indicate end of loop for timer
      timer_.loopEnd();

      // exit loop if current total run duration (since init) exceeds max_millis
      if (max_millis_ > 0 && timer_.checkDuration(max_millis_))
        break;

      // exit loop if current iterations exceeds max_iterations
      if (max_iterations_ > 0 && iterations_ > max_iterations_)
        break;
      iterations_++;
    }  // Search finish
  }
};

#endif  // MCTS_SOLVER_H
