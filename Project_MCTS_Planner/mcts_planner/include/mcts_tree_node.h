/**
 * @file    mcts_tree_node.h
 * @brief   A TreeNode in the Monte Carlo tree, which is independent of UCT/MCTS.
 *          Only contains information / methods related to State, Action, Parent, Children etc.
 */

#ifndef MCTS_TREE_NODE_H
#define MCTS_TREE_NODE_H


#include <math.h>
#include <algorithm>
#include <memory>
#include <vector>


template <class State, typename Action>
class TreeNodeT
{
  typedef std::shared_ptr<TreeNodeT<State, Action>> TreeNodePtr;

 private:
  State      state_;     // the state of this TreeNode
  Action     action_;    // the action which led to the state of this TreeNode
  TreeNodeT* parent_;    // parent of this TreeNode
  int     agent_id_;  // agent who made the decision

  int  num_visits_;  // number of times TreeNode has been visited
  float value_;       // value of this TreeNode
  int  depth_;

  std::vector<TreeNodePtr> children_;  // all current children
  std::vector<Action>      actions_;   // possible actions from this state

  //--------------------------------------------------------------
  // create a clone of the current state, apply action, and add as child
  TreeNodeT* addChildWithAction(const Action& new_action)
  {  
    clock_t t1, t2, t3, t4, t5, t6, t7;
    t1 = clock();
    // create a new TreeNode with the same state (will get cloned) as this TreeNode
    std::shared_ptr<TreeNodeT> child_node = std::make_shared<TreeNodeT>(state_, this);
    t2 = clock();
    std::cout<< "Run time for addChildWithAction (create a new TreeNode): " << double(t2 - t1) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;
    // set the action of the child to be the new action
    child_node->action_ = new_action;
    
    // apply the new action to the state of the child TreeNode
    child_node->state_.applyAction(new_action, child_node->getDepth());
    t3 = clock();
    std::cout<< "Run time for addChildWithAction (apply action): " << double(t3 - t2) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;
    // add to children
    children_.push_back(TreeNodePtr(child_node));

    t4 = clock();
    std::cout<< "Run time for addChildWithAction (add children): " << double(t4 - t3) / CLOCKS_PER_SEC * 1000 << "ms" <<std::endl;

    return child_node.get();
  }

 public:
  //--------------------------------------------------------------
  TreeNodeT(const State& state, TreeNodeT* parent = NULL)
      : state_(state), action_(), parent_(parent), agent_id_(state.agentId()), num_visits_(0), value_(0), depth_(parent ? parent->depth_ + 1 : 0)
  {
  }

  //--------------------------------------------------------------
  // expand by adding a single child
  TreeNodeT* expand()
  {
    
    // sanity check that we're not already fully expanded
    if (isFullyExpanded())
      return NULL;

    // sanity check that we don't have more children than we do actions
    // assert(children.size() < actions.size()) ;

    // if this is the first expansion and we haven't yet got all of the possible actions
    if (actions_.empty())
    {
      // retrieve list of actions from the state
      state_.getActions(actions_);

      // randomize the order
      std::random_shuffle(actions_.begin(), actions_.end());
    }

    // add the next action in queue as a child
    
    return addChildWithAction(actions_[children_.size()]);
 
  }

  //--------------------------------------------------------------
  void update(const std::vector<float>& rewards)
  {
    this->value_ += rewards[agent_id_];
    num_visits_++;
  }

  //--------------------------------------------------------------
  // GETTERS
  // state of the TreeNode
  const State& getState() const
  {          
    return state_;
  }

  // the action that led to this state
  const Action& getAction() const
  {
    return action_;
  }

  // all children have been expanded and simulated
  bool isFullyExpanded() const
  {
    return children_.empty() == false && children_.size() == actions_.size();
  }

  // does this TreeNode end the search (i.e. the game)
  bool isTerminal() const
  {
    return state_.isTerminal();
  }

  // number of times the TreeNode has been visited
  int getNumVisits() const
  {
    return num_visits_;
  }

  // accumulated value (wins)
  float getValue() const
  {
    return value_;
  }

  // how deep the TreeNode is in the tree
  int getDepth() const
  {
    return depth_;
  }

  // number of children the TreeNode has
  int getNumChildren() const
  {
    return children_.size();
  }

  // get the i'th child
  TreeNodeT* getChild(int i) const
  {
    return children_[i].get();
  }

  // get parent
  TreeNodeT* getParent() const
  {
    return parent_;
  }
};

#endif  // MCTS_TREE_NODE_H
