#include "mcts_exp3.h"

Exp3::Exp3(){}
int draw(std::vector<float> probs)
{
    float z = (float)rand()/RAND_MAX;
    float cum_prob = 0.f;
    for(int i = 0; i < probs.size(); i++)
    {
        float prob;
        prob = probs[i];
        cum_prob += prob;
        if(cum_prob > z)
        {
            return i;
        }
    }
    return probs.size() - 1;
}
void Exp3::Init(const float gamma,const int n_arms)
{
    gamma_ = gamma;
    n_arms_ = n_arms;
    std::vector<float> normalized_weights;
    for(int i = 0; i < n_arms; i++){
        normalized_weights.push_back(1);
    }
    normalized_weights_ = normalized_weights;
}