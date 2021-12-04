#include <stdexcept>
#include <random>
#include <cstring>

#include <WALL.h>
#include <PARTICLE.h>
#include <PARTICLE_SYSTEM.h>

#include "cylindrical_wall_simulation.h"


std::size_t diode_grid::index(std::size_t bin_circ, std::size_t bin_h)
{
    return (bin_circ%n_circ) + bin_h * n_circ;
}
void diode_grid::add_light(double x, double y, double z)
{
    const auto len = std::hypot(x,z);
    const auto angle = std::atan2(x/len, z/len);
    const auto angle_norm = angle/M_PI / 2 + 0.5;
    const auto circ_bin_size = 1.0 / static_cast<double>(n_circ);
    const auto circ_bin = static_cast<std::size_t>(angle_norm / circ_bin_size);
    
    const auto h_bin_size = height / static_cast<double>(n_height);
    const auto h_bin = static_cast<std::size_t>(y / h_bin_size);
    
    const auto calc_pos = [&](auto bc, auto bh)
    {
        return std::make_pair(bc*circ_bin_size, bh*h_bin_size);
    };
    
    const auto delta = [](auto a, auto b)
    {
        return a > b ? a-b : b-a;
    };
    
    const auto light = [&](auto bc, auto bh)
    {
        const auto& [pc, ph] = calc_pos(bc, bh);
        const auto dh = delta(ph, y);
        const auto dc = std::min(delta(pc, angle_norm), delta(pc+1, angle_norm));
        return brightness / std::pow(std::hypot(dh,dc),2);
    };
    const auto add = [&](auto bc, auto bh)
    {
        diodes.at(index(bc, bh)) += light(bc,bh);
    };
        
    for(std::size_t offh = 0; offh < 1 + 2*n_radius_h; ++offh )
    {
        if(offh + h_bin < n_radius_h)
        {
            //below bottom
            continue;
        }
        const auto ih = (h_bin + offh) - n_radius_h ;
        if(ih >= n_height)
        {
            //above top
            continue;
        }
        for(std::size_t offcirc = 0; offcirc < 1 + 2*n_radius_circ; ++offcirc )
        {
            const auto ic = n_circ + circ_bin - n_radius_circ + offcirc;
            add(ic, ih);
        }
    }
}
void diode_grid::clear_light()
{
    diodes.resize(n_circ*n_height);
    std::memset(diodes.data(), 0, sizeof(double)*diodes.size());
}


cylindrical_wall_simulation::cylindrical_wall_simulation(
        double radius_inner, 
        double radius_outer, 
        double height,
        double radius_particle,
        std::size_t particle_count
)
{
    _particle_system = std::make_unique<PARTICLE_SYSTEM>(SCENARIO{});
    reset(radius_inner, radius_outer, height, radius_particle, particle_count);
}

    
    void cylindrical_wall_simulation::reset(
                double radius_inner, 
                double radius_outer, 
                double height,
                double radius_particle,
                std::size_t particle_count
        )
    {
        if(radius_inner < 0)
        {
            throw std::invalid_argument{"inner radius < 0"};
        }
        if(radius_inner >= radius_outer)
        {
            throw std::invalid_argument{"inner radius >= outer radius"};
        }
        if(radius_particle <= 0)
        {
            throw std::invalid_argument{"particle radius <= 0"};
        }
        if(height <= 0)
        {
            throw std::invalid_argument{"height <= 0"};
        }
        _radius_inner=radius_inner;
        _radius_outer=radius_outer;
        _height=height;
        SCENARIO scenario_flask_wall;
        scenario_flask_wall.name = "flask_wall";
        scenario_flask_wall.boxSize.x = 2*radius_outer;
        scenario_flask_wall.boxSize.y = height;
        scenario_flask_wall.boxSize.z = 2*radius_outer;
        
        // bottom
        scenario_flask_wall.collision.walls.emplace_back(
            VEC3D(0, 1, 0),
            VEC3D(0, 0, 0));
        // top
        scenario_flask_wall.collision.walls.emplace_back(
            VEC3D(0, -1, 0),
            VEC3D(0, height, 0));
        //outer
        scenario_flask_wall.collision.cylindrical_walls.emplace_back(
            VEC3D(0, 0, 0), // center
            VEC3D(0, 1, 0), // up
            radius_outer,   // r
            true);          //contain
        //inner
        scenario_flask_wall.collision.cylindrical_walls.emplace_back(
            VEC3D(0, 0, 0), // center
            VEC3D(0, 1, 0), // up
            radius_inner,   // r
            false);         //contain
        
        std::mt19937_64 gen{std::random_device{}()};
        std::uniform_real_distribution<double> d{0,1};
        for(;particle_count;--particle_count)
        {
            const auto avgr = (_radius_inner + _radius_outer) /2;
            const auto angle =d(gen) * M_PI * 2;
            const auto x = std::sin(angle)*avgr;
            const auto z = std::cos(angle)*avgr;
            const auto y = _height * 0.1 + d(gen) * 0.8 * _height;
            scenario_flask_wall.particles.initial.push_back(PARTICLE(VEC3D(x, y, z)));
        }
        _particle_system->particle_r = radius_particle;
        _particle_system->loadScenario(scenario_flask_wall);
        
    }
    
    
void cylindrical_wall_simulation::gravity(double x, double y, double z)
{
    _particle_system->gravityVector.x=x;
    _particle_system->gravityVector.y=y;
    _particle_system->gravityVector.z=z;
}

void cylindrical_wall_simulation::step(
            double dt)
{
    _particle_system->stepVerlet(dt);
}

void cylindrical_wall_simulation::visit_particles(const std::function<void (int, const PARTICLE &)> &callback)
{
    if(callback)
    {
        const auto& grid = *(_particle_system->grid);
        for (int gridCellIndex = 0; gridCellIndex < grid.cellCount(); gridCellIndex++)
        {
            for (const auto& p : grid.data().at(gridCellIndex))
            {
                if(callback)
                {
                    callback(p.id(), p);
                }
            }
        }
    }
}
