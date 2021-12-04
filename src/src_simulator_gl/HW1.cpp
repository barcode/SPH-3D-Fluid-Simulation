///////////////////////////////////////////////////////////////////////////////
// This is an extensively reworked version of Example 3-1
// from the OpenGL Red Book:
//
// http://www.glprogramming.com/red/chapter03.html
//
///////////////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <sys/time.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "glvu/glvu.h"

#include "WALL.h"
#include "PARTICLE.h"
#include "PARTICLE_SYSTEM.h"
#include <vector>
#include <memory>

// GUI interaction stuff
GLVU glvu;

std::unique_ptr<PARTICLE_SYSTEM> particleSystem;

double dt = 1.0 / 100.0;
bool animate = false;

int iterationCount = 0;

double arUtilTimer(void);
void arUtilTimerReset(void);


SCENARIO scenario_dam;
SCENARIO scenario_cube;
SCENARIO scenario_faucet;
SCENARIO scenario_cylinder;
SCENARIO scenario_flask_wall;

///////////////////////////////////////////////////////////////////////
// draw coordinate axes
///////////////////////////////////////////////////////////////////////
void drawAxes()
{
    //glDisable(GL_COLOR_MATERIAL);
    // draw coordinate axes
    glPushMatrix();
    glTranslatef(-0.1f, -0.1f, -0.1f);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    // x axis is red
    glColor4f(10.0f, 0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glColor4f(10.0f, 0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);

    // y axis is green
    glColor4f(0.0f, 10.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glColor4f(0.0f, 10.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);

    // z axis is blue
    glColor4f(0.0f, 0.0f, 10.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glColor4f(0.0f, 0.0f, 10.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    glLineWidth(1.0f);
    glPopMatrix();
}

///////////////////////////////////////////////////////////////////////////////
// The drawing function
///////////////////////////////////////////////////////////////////////////////
void displayCallback()
{
    glvu.BeginFrame();

    // clear away the previous frame
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    //drawAxes();

    // draw the particle system and walls
    particleSystem->draw();

    // swap the buffers
    //glutSwapBuffers();

    glvu.EndFrame();
}

///////////////////////////////////////////////////////////////////////////////
// The projection function
///////////////////////////////////////////////////////////////////////////////
void reshapeCallback(int width, int height)
{
    std::cout << "reshape" << std::endl;
    // set the viewport resolution (w x h)
    glViewport(0, 0, (GLsizei) width, (GLsizei) height);

    // Make ensuing transforms affect the projection matrix
    glMatrixMode(GL_PROJECTION);

    // set the projection matrix to an orthographic view
    glLoadIdentity();
    gluPerspective(65.0, (float)width / height, 0.01, 1000.0);

    // set the matric mode back to modelview
    glMatrixMode(GL_MODELVIEW);

    // set the lookat transform
    glLoadIdentity();
    gluLookAt(0, 0, 0.5, 0, 0, 0, 0, 1, 0);
}

///////////////////////////////////////////////////////////////////////////////
// Keyboard command processing function
///////////////////////////////////////////////////////////////////////////////
void keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
    {
        // quit entirely
        case 'q':
        case 'Q':
            exit(0);
            break;

        case 'a':
            animate = !animate;
            break;

        case ' ':
            particleSystem->stepVerlet(dt);
            glutPostRedisplay();
            break;

        case 'g':
            particleSystem->toggleGridVisble();
            break;

        case 's':
            particleSystem->toggleSurfaceVisible();
            break;

        case '/':
            particleSystem->toggleGravity();
            break;

        case 't':
            particleSystem->toggleTumble();
            break;

        case 'z':
            particleSystem->toggleArrows();
            break;

        case '1':
            particleSystem->loadScenario(scenario_dam);
            break;
        case '2':
            particleSystem->loadScenario(scenario_cube);
            break;
        case '3':
            particleSystem->loadScenario(scenario_faucet);
            break;
        case '4':
            particleSystem->loadScenario(scenario_cylinder);
            break;
        case '5':
            particleSystem->loadScenario(scenario_flask_wall);
            break;

        case 'f':
            printf("*** %f (frame/sec)\n", (double)iterationCount / arUtilTimer());
            break;
    }
    glvu.Keyboard(key, x, y);

    glutPostRedisplay();

}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
void glutMouseClick(int button, int state, int x, int y)
{
    glvu.Mouse(button, state, x, y);
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
void glutMouseMotion(int x, int y)
{
    glvu.Motion(x, y);
    glvuVec3f viewVector = glvu.GetCurrentCam()->Y;
    particleSystem->setGravityVectorWithViewVector(VEC3D(viewVector[0], viewVector[1], viewVector[2]));
}

///////////////////////////////////////////////////////////////////////////////
// Idle command processing function
///////////////////////////////////////////////////////////////////////////////
void idleCallback()
{
    if (!animate)
    {
        return;
    }
    if (iterationCount == 0)
    {
        arUtilTimerReset();
    }
    particleSystem->stepVerlet(dt);
    iterationCount++;

    glutPostRedisplay();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    //set up scenarios
    {
        //dam
        {
            scenario_dam.name = "dam";
            scenario_dam.boxSize.x = 0.4 * 2.0;
            scenario_dam.boxSize.y = 0.4;
            scenario_dam.boxSize.z = 0.4 / 2.0;
            scenario_dam.generate_box_walls();

            for (double y = -scenario_dam.boxSize.y / 2.0; y < scenario_dam.boxSize.y / 2.0; y += h / 2.0)
            {
                for (double x = -scenario_dam.boxSize.x / 2.0; x < -scenario_dam.boxSize.x / 4.0; x += h / 2.0)
                {
                    for (double z = -scenario_dam.boxSize.z / 2.0; z < scenario_dam.boxSize.z / 2.0; z += h / 2.0)
                    {
                        scenario_dam.particles.initial.push_back(PARTICLE(VEC3D(x, y, z)));
                    }
                }
            }
        }
        //cube
        {
            scenario_cube.name = "cube";
            scenario_cube.boxSize.x = 0.4;
            scenario_cube.boxSize.y = 0.4;
            scenario_cube.boxSize.z = 0.4;
            scenario_cube.generate_box_walls();

            for (double y = 0; y < scenario_cube.boxSize.y; y += h / 2.0)
            {
                for (double x = -scenario_cube.boxSize.x / 4.0; x < scenario_cube.boxSize.x / 4.0; x += h / 2.0)
                {
                    for (double z = -scenario_cube.boxSize.z / 4.0; z < scenario_cube.boxSize.z / 4.0; z += h / 2.0)
                    {
                        scenario_cube.particles.initial.push_back(PARTICLE(VEC3D(x, y, z)));
                    }
                }
            }
        }
        //faucet
        {
            static constexpr double sx = 0.4;
            static constexpr double sy = 0.4;
            static constexpr double sz = 0.4;
            scenario_faucet.name = "faucet";
            scenario_faucet.boxSize.x = sx;
            scenario_faucet.boxSize.y = sy;
            scenario_faucet.boxSize.z = sz;
            scenario_faucet.generate_box_walls();

            scenario_faucet.particles.generator = [](auto & sys)
            {
                if (PARTICLE::count < 3000)
                {
                    VEC3D initialVelocity(-1.8, -1.8, 0);

                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy + h * 0.6, 0), initialVelocity);
                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy, 0), initialVelocity);
                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy + h * -0.6, 0), initialVelocity);

                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy + h * 0.3, h * 0.6), initialVelocity);
                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy + h * 0.3, h * -0.6), initialVelocity);

                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy + h * -0.3, h * 0.6), initialVelocity);
                    sys.addParticle(VEC3D(sx / 2.0 - h / 2.0, sy + h * -0.3, h * -0.6), initialVelocity);
                }
            };
        }
        //cylinder
        {
            scenario_cylinder.name = "cylinder";
            scenario_cylinder.boxSize.x = 0.4;
            scenario_cylinder.boxSize.y = 0.4;
            scenario_cylinder.boxSize.z = 0.4;

            for (double y = 0; y < scenario_cylinder.boxSize.y; y += h / 2.0)
            {
                for (double x = -scenario_cylinder.boxSize.x / 4.0; x < scenario_cylinder.boxSize.x / 4.0; x += h / 2.0)
                {
                    for (double z = -scenario_cylinder.boxSize.z / 4.0; z < scenario_cylinder.boxSize.z / 4.0; z += h / 2.0)
                    {
                        scenario_cylinder.particles.initial.push_back(PARTICLE(VEC3D(x, y, z)));
                    }
                }
            }
            // bottom
            scenario_cylinder.collision.walls.emplace_back(
                VEC3D(0, 1, 0),
                VEC3D(0, -scenario_cylinder.boxSize.y / 2.0, 0));
            //outer
            scenario_cylinder.collision.cylindrical_walls.emplace_back(
                VEC3D(0, -scenario_cylinder.boxSize.y / 2.0, 0),
                VEC3D(0, 1, 0),
                scenario_cylinder.boxSize.x / 2,
                true);
        }
        //flask_wall
        {
            scenario_flask_wall.name = "flask_wall";
            scenario_flask_wall.boxSize.x = 0.4;
            scenario_flask_wall.boxSize.y = 1;
            scenario_flask_wall.boxSize.z = 0.4;

            for (double y = -scenario_flask_wall.boxSize.y / 2 ; y < scenario_flask_wall.boxSize.y / 2; y += h)
            {
                for (double x = -scenario_flask_wall.boxSize.x / 3.0; x < scenario_flask_wall.boxSize.x / 3.0; x += h / 2.0)
                {
                    for (double z = -scenario_flask_wall.boxSize.z / 3.0; z < scenario_flask_wall.boxSize.z / 3.0; z += h / 2.0)
                    {
                        if (std::hypot(x, z) - 0.01 > scenario_flask_wall.boxSize.x / 4)
                        {
                            scenario_flask_wall.particles.initial.push_back(PARTICLE(VEC3D(x, y, z)));
                        }
                    }
                }
            }
            // bottom
            scenario_flask_wall.collision.walls.emplace_back(
                VEC3D(0, 1, 0),
                VEC3D(0, -scenario_flask_wall.boxSize.y / 2.0, 0));
            // top
            scenario_flask_wall.collision.walls.emplace_back(
                VEC3D(0, -1, 0),
                VEC3D(0, scenario_flask_wall.boxSize.y / 2.0, 0));
            //outer
            scenario_flask_wall.collision.cylindrical_walls.emplace_back(
                VEC3D(0, 0, 0),
                VEC3D(0, 1, 0),
                scenario_flask_wall.boxSize.x / 2,
                true);
            //inner
            scenario_flask_wall.collision.cylindrical_walls.emplace_back(
                VEC3D(0, 0, 0),
                VEC3D(0, 1, 0),
                scenario_flask_wall.boxSize.x / 4,
                false);
        }
    }

    char title[] = "sph";

    glutInit(&argc, argv);
    glvu.Init(title, GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH, 0, 0, 800, 800);
    glShadeModel(GL_SMOOTH);

    glvu.SetInertiaEnabled(0);

    // point GLUT to our callback functions
    glutDisplayFunc(displayCallback);
    glutIdleFunc(idleCallback);
    //glutReshapeFunc(reshapeCallback);
    glutKeyboardFunc(keyboardCallback);
    glutMouseFunc(glutMouseClick);
    glutMotionFunc(glutMouseMotion);


    // set background to black
    glClearColor(1.0, 1.0, 1.0, 1.0);

    // enable lights
    GLfloat ambient[] = {0.7, 0.7, 0.7};
    GLfloat diffuse[] = {1.0, 1.0, 1.0};
    GLfloat specular[] = {0.0, 0.0, 0.0};
    //GLfloat lightPosition[] = { 0.0, 2.0, 0.0 };

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    //glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glvuVec3f ModelMin(-10, -10, -10), ModelMax(10, 10, 10),
              Eye(0, 0, 1.5),  LookAtCntr(0, 0, 0),  Up(0, 1, 0);

    float Yfov = 45;
    float Aspect = 1;
    float Near = 0.001f;
    float Far = 10.0f;
    glvu.SetAllCams(ModelMin, ModelMax, Eye, LookAtCntr, Up, Yfov, Aspect, Near, Far);

    glvuVec3f center(0.0, 0.0, 0.0);
    glvu.SetWorldCenter(center);

    particleSystem = std::make_unique<PARTICLE_SYSTEM>(scenario_dam);

    // Let GLUT take over
    glutMainLoop();

    return 0;
}

static int      ss, sms;

double arUtilTimer(void)
{
#ifdef _WIN32
    struct _timeb sys_time;
    double             tt;
    int                s1, s2;

    _ftime(&sys_time);
    s1 = sys_time.time  - ss;
    s2 = sys_time.millitm - sms;
#else
    struct timeval     time;
    double             tt;
    int                s1, s2;

#if defined(__linux) || defined(__APPLE__)
    gettimeofday(&time, NULL);
#else
    gettimeofday(&time);
#endif
    s1 = time.tv_sec  - ss;
    s2 = time.tv_usec / 1000 - sms;
#endif

    tt = (double)s1 + (double)s2 / 1000.0;

    return (tt);
}

void arUtilTimerReset(void)
{
#ifdef _WIN32
    struct _timeb sys_time;

    _ftime(&sys_time);
    ss  = sys_time.time;
    sms = sys_time.millitm;
#else
    struct timeval     time;

#if defined(__linux) || defined(__APPLE__)
    gettimeofday(&time, NULL);
#else
    gettimeofday(&time);
#endif
    ss  = time.tv_sec;
    sms = time.tv_usec / 1000;
#endif
}

