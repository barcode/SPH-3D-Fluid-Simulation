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

#include "cylindrical_wall_simulation.h"

// GUI interaction stuff
GLVU glvu;

std::unique_ptr<cylindrical_wall_simulation> sim;

double dt = 1.0 / 100.0;
bool animate = false;

int iterationCount = 0;

double arUtilTimer(void);
void arUtilTimerReset(void);

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
    sim->_particle_system->draw();

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
            sim->step(dt);
            glutPostRedisplay();
            break;

        case 'g':
            sim->_particle_system->toggleGridVisble();
            break;

        case '=':
            sim->_particle_system->SURFACE_THRESHOLD += 0.1;
            std::cout << "surface threshold: " << sim->_particle_system->SURFACE_THRESHOLD << std::endl;
            break;

        case '-':
            sim->_particle_system->SURFACE_THRESHOLD -= 0.1;
            std::cout << "surface threshold: " << sim->_particle_system->SURFACE_THRESHOLD << std::endl;
            break;


    case 'c':
        sim->_catch_escaped_particles = !sim->_catch_escaped_particles;
        std::cout << "catch escaped particles " << sim->_catch_escaped_particles << "\n";
        break;



        case 's':
            sim->_particle_system->toggleSurfaceVisible();
            break;

        case '/':
            sim->_particle_system->toggleGravity();
            break;

        case 't':
            sim->_particle_system->toggleTumble();
            break;

        case 'z':
            sim->_particle_system->toggleArrows();
            break;

        case '1':
            sim->reset(0.2, //radius_inner
                       0.5 , //radius_outer
                       1, //height
                       0.1, //radius_particle
                       1000// particle_count
                       );
            break;
        case '2':
        sim->reset(0.2, //radius_inner
                   0.5 , //radius_outer
                   1, //height
                   0.1, //radius_particle
                   5000// particle_count
                   );
            break;
        case '3':
        sim->reset(0.3, //radius_inner
                   0.5 , //radius_outer
                   1, //height
                   0.1, //radius_particle
                   1000// particle_count
                   );
            break;
    case '4':
    sim->reset(0.15, //radius_inner
               0.3 , //radius_outer
               1, //height
               0.1, //radius_particle
               2500// particle_count
               );
        break;
    case '5':
    sim->reset(0.2, //radius_inner
               0.5 , //radius_outer
               1, //height
               0.2, //radius_particle
               5000// particle_count
               );
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
    sim->_particle_system->setGravityVectorWithViewVector(VEC3D(viewVector[0], viewVector[1], viewVector[2]));
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
    sim->step(dt);
    iterationCount++;

    glutPostRedisplay();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
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

    sim = std::make_unique<cylindrical_wall_simulation>(
                0.2, //radius_inner
                0.5 , //radius_outer
                1, //height
                0.1, //radius_particle
                1000// particle_count
                );

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

