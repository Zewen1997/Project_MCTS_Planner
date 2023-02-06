#ifndef MCTS_EXP3_H
#define MCTS_EXP3_H

#include <cmath>
#include <numeric>
#include <vector>
#include <set>
#include <iostream>
#include <algorithm>

class Exp3
{
private:
    float gamma_;
    int n_arms_;
    std::vector<float> normalized_weights_;
    

public:
    Exp3();
    void init(const float gamma, const int n_arms);
    int selectArm();
    void update(const int chosen_arm, const float reward);
    std::vector<float>& normalize(std::vector<float>& data);
    void printWeights();

};

#endif