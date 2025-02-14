#pragma once

#include <functional>

#include "PARTICLE.h"
#include "WALL.h"
#include "CYLINDRICAL_WALL.h"
#include <vector>
#include "FIELD_3D.h"


class PARTICLE_SYSTEM;

struct SCENARIO
{
    std::string           name="default";
    VEC3D                 boxSize{1,1,1};

    struct PARTICLES
    {
        std::vector<PARTICLE>                 initial;
        std::function<void(PARTICLE_SYSTEM&)> generator;
    };
    PARTICLES particles;

    struct COLLISION
    {
        std::vector<WALL>               walls;
        std::vector<CYLINDRICAL_WALL>   cylindrical_walls;
    };
    COLLISION collision;

    void generate_box_walls();
};

class PARTICLE_SYSTEM
{
public:
    PARTICLE_SYSTEM(const SCENARIO& s);
    ~PARTICLE_SYSTEM();

    void updateGrid();
#ifndef NO_OPENGL
    // draw to OGL
    void draw();
#endif

    void addParticle(const VEC3D& position);

    void addParticle(const VEC3D& position, const VEC3D& velocity);

    void stepVerlet(double dt);

    void calculateAcceleration();

    void collisionForce(PARTICLE& particle, VEC3D& f_collision);

    double Wpoly6(double radiusSquared);

    void Wpoly6Gradient(VEC3D& diffPosition, double radiusSquared, VEC3D& gradient);

    double Wpoly6Laplacian(double radiusSquared);

    void WspikyGradient(VEC3D& diffPosition, double radiusSquared, VEC3D& gradient);

    double WviscosityLaplacian(double radiusSquared);

    void toggleGridVisble();

    void toggleSurfaceVisible();

    void toggleGravity();

    void toggleArrows();

    void toggleTumble();

    void setGravityVectorWithViewVector(VEC3D viewVector);

    void loadScenario(const SCENARIO& s);

    //typedef std::tr1::tuple<int,int,int> gridKey;
    //std::map<gridKey, std::vector<PARTICLE> > grid;

    static constexpr double GRAVITY_ACCELERATION = -9.80665;

    FIELD_3D* grid;
    VEC3D gravityVector{0,GRAVITY_ACCELERATION,0};
    double particle_r = 0.01;
    double h = 0.0457; //0.02 //0.045

    double GAS_STIFFNESS = 3.0; //20.0 // 461.5  // Nm/kg is gas constant of water vapor
    double REST_DENSITY = 998.29; // kg/m^3 is rest density of water particle
    double PARTICLE_MASS = 0.02; // kg
    double VISCOSITY = 3.5; // 5.0 // 0.00089 // Ns/m^2 or Pa*s viscosity of water
    double SURFACE_TENSION = 0.0728; // N/m
    double SURFACE_THRESHOLD = 7.065;
    double KERNEL_PARTICLES = 20.0;


    double WALL_K = 10000.0; // wall spring constant
    double WALL_DAMPING = -0.9; // wall damping constant

    double BOX_SIZE = 0.4;

    unsigned int num_particles() const
    {
        return _num_particles;
    }
private:
    // list of particles, walls, and springs being simulated
    SCENARIO _scenario;

    //unsigned int _particleCount;
    bool _isGridVisible;
    bool _tumble;
    unsigned int _iteration = 0;
    unsigned int _num_particles = 0;
};
