#include "mcts_exp3.h"
int seed_ = 0;
Exp3::Exp3(){}

Exp3::Exp3(){}
int draw(std::vector<float> probs)
{   
    srand(seed_);
    seed_ += 5;
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
void Exp3::init(const float gamma,const int n_arms)
{
    gamma_ = gamma;
    n_arms_ = n_arms;
    std::vector<float> normalized_weights;
    for(int i = 0; i < n_arms_; i++){
        normalized_weights.push_back(1.0f / float(n_arms_));
    }
    normalized_weights_ = normalized_weights;
    std::cout<<"init_weights: "<<normalized_weights_[0]<<std::endl;
}

int Exp3::selectArm()
{
    float total_weight = std::accumulate(normalized_weights_.begin(), normalized_weights_.end(), 0.0f);
    std::vector<float> probs;
    for(int i = 0; i < n_arms_; i++)
    {
        probs.push_back(0.0f);
    }
    for(int arm = 0; arm < n_arms_; arm++)
    {
        probs[arm] = (1 - gamma_) * (normalized_weights_[arm] / total_weight);
        probs[arm] = probs[arm] + gamma_ * (1.0f / float(n_arms_));
    }
    return draw(probs);
}

std::vector<float>& Exp3::normalize(std::vector<float>& data)
{
    std::set<float> s;
    float minValue = *min_element(data.begin(), data.end());
    for(int i = 0; i < data.size(); i++)
    {
        s.insert(data[i]);
    }
    if(s.size() != 1)
    { 
        for(float& x : data)
            x -= minValue;
        float sum = std::accumulate(data.begin(), data.end(), 0.0f);
        for(float& x : data)
            x /= sum;
        return data;
    }
    else
    {
        return data;
    }
}

void Exp3::update(int chosen_arm, float reward)
{
    float total_weight = std::accumulate(normalized_weights_.begin(), normalized_weights_.end(), 0.0f);
    std::vector<float> probs(n_arms_, 0.0f);

    for(int arm = 0; arm < n_arms_; arm++)
    {
        probs[arm] = (1 - gamma_) * (normalized_weights_[arm] / total_weight);
        probs[arm] = probs[arm] + gamma_ * (1.0f / float(n_arms_));
    }
    float x = reward / probs[chosen_arm];
    std::cout<<"x: "<<x<<std::endl;
    float growth_factor = exp((gamma_ / n_arms_) * x);
    std::cout<<"growth_factor>: "<<growth_factor<<std::endl;
    normalized_weights_[chosen_arm] = normalized_weights_[chosen_arm] * growth_factor;

    // When the weight of each arm is different, perform normalization, otherwise there will be extreme cases where one of the values
    // is and the other values are 0! 
    std::set<float> s;
    for(int i = 0; i < normalized_weights_.size(); i++)
    {
        s.insert(normalized_weights_[i]);
    }
    if (s.size() == normalized_weights_.size())
    {
        normalized_weights_ = this->normalize(normalized_weights_);
    }

}

void Exp3::printWeights()
{   
    for (int i = 0; i < normalized_weights_.size(); i++)
    {
        std::cout<<"weight "<< i << ":"<< normalized_weights_[i]<<std::endl;
    }
}
