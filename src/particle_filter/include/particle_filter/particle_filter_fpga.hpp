#ifndef PARTICLE_FILTER_PARTICLE_FILTER_FGPA_HPP
#define PARTICLE_FILTER_PARTICLE_FILTER_FGPA_HPP

#include "particle_filter/particle_filter.hpp"

class ParticleFilterFPGA : public ParticleFilter
{
  public:
    ParticleFilterFPGA();
    virtual void normalize(); // TODO: accelerate this
};

#endif // PARTICLE_FILTER_PARTICLE_FILTER_FGPA_HPP
