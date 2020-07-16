#ifndef UTILS_SAMPLER_H
#define UTILS_SAMPLER_H

#include <cstddef>
#include <array>

static const std::size_t SAMPLE_SIZE = 5;

class Sampler {
public:
    Sampler();
    void add(float sample);
    float get_average()const;
private:
    std::array<float, SAMPLE_SIZE> samples;
    size_t next;
};

#endif
