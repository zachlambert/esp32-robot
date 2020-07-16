#include "sampler.hpp"

#include <numeric>

Sampler::Sampler():samples(), next(0)
{
    // Array is not zero initialised
    samples.fill(0);
}

void Sampler::add(float sample)
{
    samples[next] = sample; 
    next = (next+1) % samples.size();
}

float Sampler::get_average()const
{
    return std::accumulate(
        samples.begin(), samples.end(), (float)0
    ) / samples.size();
}
