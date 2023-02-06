/**
 * @file    mcts_solver.h
 * @brief   Implementation of a Monte Carlo Tree Search (MCTS) Solver.
 */

#ifndef MCTS_SOLVER_H
#define MCTS_SOLVER_H

#include <cfloat>
#include <numeric>
#include "mcts_timer.h"
#include "mcts_tree_node.h"
#include "mcts_exp3.h"
#include "mcts_planner.h"


template <class State, typename Action>
class MCTS
{
  typedef TreeNodeT<State, Action> TreeNode;

 private:
          
  LoopTimer                 timer_;
  int                       iterations_;
  // std::vector<float>        to_be_normalized_score_ = {FLT_EPSILON};
  std::vector<float>        strategy_set_ = {0.001f, 0.1f, 1.f, 10.f, 1000.f};
  std::shared_ptr<TreeNode> root_node_;  // root node of tree 
  int                       chosen_arm_;

  

 public:
  float uct_k_;             // k value in UCT function. default = sqrt(2)
  int  max_iterations_;    // do a maximum of this many iterations (0 to run till end)
  int  max_millis_;        // run for a maximum of this many milliseconds (0 to run till end)
  int  simulation_depth_;  // how many ticks (frames) to run simulation for
  int  max_tree_depth_;    // maximum of tree depth
  std::shared_ptr<MCTSDataHandler>   mcts_data_handler_;
  std::shared_ptr<Exp3>     mcts_exp3_;
  std::vector<float>        to_be_normalized_score_ = {FLT_EPSILON}; 

  //--------------------------------------------------------------
  MCTS() : iterations_(0), chosen_arm_(0), uct_k_(sqrt(2)), max_iterations_(100), max_millis_(0), simulation_depth_(10), max_tree_depth_(10) {  
    mcts_exp3_ = std::make_shared<Exp3>();
    mcts_exp3_->init(0.3, strategy_set_.size());
    }

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

    while (choose_node != NULL && !choose_node->isTerminal() && choose_node->getNumChildren() > 0)
    {
      choose_node = getBestUCTCchild(choose_node, uct_k_);
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

    while (choose_node != NULL && !choose_node->isTerminal() && choose_node->getNumChildren() > 0)
    {
      choose_node = getBestUCTCchild(choose_node, uct_k_);
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


    // initialize exp3
    // mcts_exp3_ = std::make_unique<Exp3>();
    // mcts_exp3_->init(0.4, strategy_set_.size());


    TreeNode* best_node = NULL;

    // iterate
    iterations_ = 0;
    while (true)
    {
      clock_t t1, t2, t3, t4, t5, t6, t7;

      // indicate start of loop
      timer_.loopStart();

      // 1. SELECT. Start at root, dig down into tree using UCT on all fully expanded nodes
      TreeNode* node = root_node_.get();
      std::vector<float> score;
      /*
      while (!node->isTerminal() && node->isFullyExpanded())
      {
        node = getBestUCTCchild(node, uct_k_);
        //	assert(node);	// sanity check
      }
      */
      
      t1 = clock();

      
      while (!node->isTerminal())
      { 
        if (node->isFullyExpanded())
        {
          node = getBestUCTCchild(node, strategy_set_[chosen_arm_]);
          score.push_back(node->getValue() / float(node->getNumVisits()));
          if(!node->isFullyExpanded())
          { 
            mcts_exp3_->printWeights();
            float mean_score = std::accumulate(score.begin(), score.end(), 0.0f) / score.size();
            to_be_normalized_score_.push_back(mean_score);
            float minValue = *min_element(to_be_normalized_score_.begin(), to_be_normalized_score_.end());
            float maxValue = *max_element(to_be_normalized_score_.begin(), to_be_normalized_score_.end());
            std::cout<<"mean_score: "<<mean_score<<std::endl;
            std::cout<<"min: "<<minValue<<std::endl;
            std::cout<<"max: "<<maxValue<<std::endl;
            float normalized_score = (mean_score - minValue) / (maxValue - minValue);
            std::cout<<"normalized_score: "<<normalized_score<<std::endl;
            mcts_exp3_->update(chosen_arm_, normalized_score);
            chosen_arm_ = mcts_exp3_->selectArm();
            std::cout<<"chosen_arm: "<<chosen_arm_<<std::endl;
          }
        }
        else
          break;
      }
      

      t2 = clock();
      std::cout<< "Run time for first step search(Using exp3): " << double(t2 - t1) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;  
      
      // 2. EXPAND by adding a single child (if not terminal or not fully expanded)
      if (!node->isFullyExpanded() && !node->isTerminal() && node->getDepth() <= max_tree_depth_)
        node = node->expand();
      
      State state(node->getState(), mcts_data_handler_);
      
      t3 = clock();
      std::cout<< "Run time for second step search(Expand a single child): " << double(t3 - t2) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;

      // 3. SIMULATE (if not terminal)
      float reward = 0.f;

      if (!node->isTerminal())
      {
        Action action;
        for (int t = 0; t < simulation_depth_; t++)
        {
          if (state.isTerminal())
            break;
          /*
          if (state.getRandomAction(action))
          { 
            
            // current depth is node depth + simulation depth
            int current_depth = node->getDepth() + t;

            state.applyAction(action, current_depth);

            float reward = state.evaluateRewards()[0];
            
            std::cout<<"reward: "<< reward <<std::endl;
          }
          else
            break;
          */
          action = state.getRandomAction();
          int current_depth = node->getDepth() + t;
          state.applyAction(action, current_depth);
          reward += state.evaluateRewards(action)[0];
          std::cout<<"reward: "<< reward <<std::endl;
        }
      }

      // get rewards vector for all agents
      const std::vector<float> rewards (1, reward);

      // add to history
      if (explored_states)
        explored_states->push_back(state);

      t4 = clock();
      std::cout<< "Run time for third step search(Simulate): " << double(t4 - t3) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;

      // 4. BACK PROPAGATION
      while (node)
      {
        node->update(rewards);
        node = node->getParent();
      }

      // find most visited child
      // best_node = getMostVisitedChild(root_node_.get());
      best_node = getBestUCTCchild(root_node_.get(), uct_k_);
      

      // indicate end of loop for timer
      timer_.loopEnd();

      // exit loop if current total run duration (since init) exceeds max_millis
      if (max_millis_ > 0 && timer_.checkDuration(max_millis_))
        break;

      // exit loop if current iterations exceeds max_iterations
      if (max_iterations_ > 0 && iterations_ > max_iterations_)
        break;
      iterations_++;

      t5 = clock();
      std::cout<< "Run time for fourth step search(Backpropogation): " << double(t5 - t4) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;
    }  // Search finish
  }
};

#endif  // MCTS_SOLVER_H
