#pragma once

#include <vector>
#include <memory>
#include <functional>

#include "PARTICLE.h"

class PARTICLE_SYSTEM;
class SCENARIO;

struct diode_grid
{
    std::size_t index(std::size_t bin_circ, std::size_t bin_h);
    void add_light(double x, double y, double z);
    void clear_light();
    std::size_t n_circ;
    std::size_t n_height;
    double brightness;
    std::vector<double> diodes;
    double height;
    std::size_t n_radius_h    = 3;
    std::size_t n_radius_circ = 3;
};

class cylindrical_wall_simulation
{
public:    
    cylindrical_wall_simulation(
            double radius_inner, 
            double radius_outer, 
            double height,
            double radius_particle,
            std::size_t particle_count
    );
    void reset(
                double radius_inner, 
                double radius_outer, 
                double height,
                double radius_particle,
                std::size_t particle_count
        );
    
    void gravity(double x, double y, double z);
    void step(double dt);    
    
    void visit_particles(const std::function<void (int, const PARTICLE &)> &callback);
    void visit_particles_mod(const std::function<void (int, PARTICLE &)> &callback);

    double _radius_inner=0.3;
    double _radius_outer=0.5;
    double _height = 1;
    bool _catch_escaped_particles = false;
     
     std::unique_ptr<PARTICLE_SYSTEM> _particle_system;
};
