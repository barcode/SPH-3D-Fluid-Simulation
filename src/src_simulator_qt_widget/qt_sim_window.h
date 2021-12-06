#pragma once

#include <memory>
#include <chrono>

#include <QMainWindow>
#include <Qt3DCore>
#include <Qt3DRender>
#include <Qt3DInput>
#include <Qt3DLogic>
#include <Qt3DExtras>
#include <QGuiApplication>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DCore/QAspectEngine>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QMaterial>

#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongAlphaMaterial>


#include "cylindrical_wall_simulation.h"

namespace Ui {
class qt_sim_window;
}

class cylindrical_wall_simulation;

class qt_sim_window : public QMainWindow
{
    Q_OBJECT

public:
    using clock_t = std::chrono::high_resolution_clock;
    static const clock_t::time_point now()
    {
        return clock_t::now();
    }
    static constexpr float dt_ms(clock_t::time_point start,
                                               clock_t::time_point stop)
    {
        const double ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop-start).count();
        return ns / 1000 / 1000;
    }

    explicit qt_sim_window(QWidget *parent = nullptr);
    ~qt_sim_window();

    void timerEvent(QTimerEvent *event) override;

    void step();

public slots:
    void reset_sim();

public:
    clock_t::time_point timer_last_End;

    Ui::qt_sim_window *ui;

    Qt3DExtras::Qt3DWindow* view;
    Qt3DCore::QEntity* scene;
    Qt3DRender::QCamera *camera;
    Qt3DExtras::QOrbitCameraController* manipulator;
    Qt3DExtras::QPhongMaterial* material_sphere;
    Qt3DExtras::QPhongAlphaMaterial* material_other;
    std::vector<Qt3DCore::QEntity*> elements;
    std::vector<Qt3DCore::QTransform*> transforms;
    std::unique_ptr<cylindrical_wall_simulation> sim;

    diode_grid _diode_grid;
    bool no_autostep = false;
};

