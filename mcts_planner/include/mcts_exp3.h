#ifndef MCTS_EXP3_H
#define MCTS_EXP3_H

#include <cmath>
#include <vector>

class Exp3
{
private:
    float gamma_;
    int n_arms_;
    std::vector<float> normalized_weights_;

public:
    Exp3();
    void Init(const float gamma,const int n_arms);
    int SelectArm();
    void Update();
    std::vector<float> Normalize();

};


#endif