#include <chrono>
#include <iostream>
#include <iomanip>

#include "PARTICLE_SYSTEM.h"
#undef h


#include "qt_sim_window.h"
#include "ui_qt_sim_window.h"


qt_sim_window::qt_sim_window(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::qt_sim_window)
{
    ui->setupUi(this);
    
    
    view = new Qt3DExtras::Qt3DWindow;
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0xffffff)));
    QWidget* container = QWidget::createWindowContainer(view);
    ui->gridLayout_scene->addWidget(container);
    
    scene = new Qt3DCore::QEntity;

    // camera
    camera = view->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0, 0, 10.0f));
    camera->setViewCenter(QVector3D(0, 0, 0));

    // manipulator
    manipulator = new Qt3DExtras::QOrbitCameraController(scene);
    manipulator->setLinearSpeed(10.f);
    manipulator->setLookSpeed(180.f);
    manipulator->setCamera(camera);
    
    material_sphere = new Qt3DExtras::QPhongMaterial(scene);
    material_sphere->setAmbient(QColor{100,100,100});

    material_other = new Qt3DExtras::QPhongAlphaMaterial(scene);
    material_other->setAmbient(QColor{0,0,100,50});
    material_other->setAlpha(0.4);
    
    view->setRootEntity(scene);
    
    
    reset_sim();
    
    startTimer(std::chrono::milliseconds{10});
    
    timer_last_End= now();
    
    connect(ui->pushButton_reset_sim, &QPushButton::clicked, 
            this, &qt_sim_window::reset_sim);
}

qt_sim_window::~qt_sim_window()
{
    delete ui;
}

void qt_sim_window::timerEvent(QTimerEvent*)
{
    const auto height = ui->doubleSpinBox_h->value();
    camera->setViewCenter({0,height/2,0});
    
    const auto timer_start= now();
    double dt_since_last = dt_ms(timer_last_End, timer_start);
    double dt_step = 0;
    double dt_diodes_image = 0;
    double dt_update_spheres = 0;
    double dt_update_light = 0;
    std::size_t npart_visited = 0;
    
    if(sim)
    {
        sim->_catch_escaped_particles = ui->checkBox_catch_parts->isChecked();
        
        sim->_particle_system->SURFACE_THRESHOLD = ui->doubleSpinBox_surf_thresh->value();
        
        sim->_particle_system->GAS_STIFFNESS    = ui->doubleSpinBox_GAS_STIFFNESS   ->value();
        sim->_particle_system->REST_DENSITY     = ui->doubleSpinBox_REST_DENSITY    ->value();
        sim->_particle_system->PARTICLE_MASS    = ui->doubleSpinBox_PARTICLE_MASS   ->value();
        sim->_particle_system->VISCOSITY        = ui->doubleSpinBox_VISCOSITY       ->value();
        sim->_particle_system->SURFACE_TENSION  = ui->doubleSpinBox_SURFACE_TENSION ->value();
        sim->_particle_system->KERNEL_PARTICLES = ui->doubleSpinBox_KERNEL_PARTICLES->value();
        sim->_particle_system->WALL_K           = ui->doubleSpinBox_WALL_K          ->value();
        sim->_particle_system->WALL_DAMPING     = ui->doubleSpinBox_WALL_DAMPING    ->value();
        
        const auto diodes_active = ui->checkBox_active_diodes->isChecked();
        if(diodes_active)
        {
            _diode_grid.n_circ        = ui->spinBox_d_nc->value();
            _diode_grid.n_height      = ui->spinBox_d_nh->value();
            _diode_grid.brightness    = ui->doubleSpinBox_d_brightness->value();
            _diode_grid.n_radius_h    = ui->spinBox_d_rh->value();
            _diode_grid.n_radius_circ = ui->spinBox_d_rc->value();
            _diode_grid.clear_light();
        }
        
        sim->gravity(ui->doubleSpinBox_grav_x->value(), 
                    ui->doubleSpinBox_grav_y->value(), 
                    ui->doubleSpinBox_grav_z->value());
        if(!no_autostep && ui->checkBox_active_sim->isChecked())
        {
            const auto step_start = now();
            step();
            dt_step = dt_ms(step_start,now());
        }
        sim->visit_particles([&](int idx, const auto& part)
        {
            ++npart_visited;
            const auto& p = part.position();
            //update pos
            {
                const auto set_sphere =now();
                auto& t = *transforms.at(idx);
                t.setTranslation({static_cast<float>(p.x),
                                  static_cast<float>(p.y),
                                  static_cast<float>(p.z)});
                dt_update_spheres += dt_ms(set_sphere,now());
            }
            //update light
            if(diodes_active)
            {
                const auto add_light =now();
                _diode_grid.add_light(p.x,p.y,p.z);
                dt_update_light += dt_ms(add_light,now());
            }
        });
        
        if(diodes_active)
        {
            const auto ds = now();
            //diodes
            QImage im(ui->spinBox_d_nc->value(), ui->spinBox_d_nh->value(), QImage::Format_Grayscale8);
            im.fill(0);
            
            for(std::size_t i = 0; i < _diode_grid.diodes.size();++i)
            {
                const int x = i % _diode_grid.n_circ;
                const int y = _diode_grid.n_circ -1 - (i / _diode_grid.n_circ);
                const int val = std::min(255.0, _diode_grid.diodes.at(i) / 100);
                im.setPixel(x,y, QColor{val,val,val}.rgb());
            }
            QPixmap px;
            px.convertFromImage(im);
            ui->label_image->setPixmap(px);
            dt_diodes_image = dt_ms(ds, now());
        }
    }
    timer_last_End= now();
    double dt_timer = dt_ms(timer_start, timer_last_End);
    std::cout << std::setprecision(3) << std::setw(8) << std::right << dt_timer << " dt_timer, "
              << std::setprecision(3) << std::setw(8) << std::right << dt_since_last<< " dt_since_last, "
              << std::setprecision(3) << std::setw(8) << std::right << dt_step<< " dt_step, "
              << std::setprecision(3) << std::setw(8) << std::right << dt_update_spheres<< " dt_update_spheres, "
              << std::setprecision(3) << std::setw(8) << std::right << dt_update_light<< " dt_update_light, "
              << std::setprecision(3) << std::setw(8) << std::right << dt_diodes_image<< " dt_diodes_image "
              << "\n";
}

void qt_sim_window::step()
{
    sim->step(ui->doubleSpinBox_dt->value());
}

void qt_sim_window::reset_sim()
{
    for(auto* sp : elements)
    {
        sp->~QEntity();
    }
    elements.clear();
    transforms.clear();
    sim.reset();
    const auto npart = static_cast<std::size_t>(ui->spinBox_npart->value());
    const auto rpart = ui->doubleSpinBox_r_part->value();
    const auto rinner = ui->doubleSpinBox_r_inner->value();
    const auto router = ui->doubleSpinBox_r_outer->value();
    const auto height = ui->doubleSpinBox_h->value();
    _diode_grid.height = height;
    std::cout << "npart " << npart << "\n";
    std::cout << "rpart " << rpart << "\n";
    sim = std::make_unique<cylindrical_wall_simulation>(
                std::min(rinner, router),
                std::max(rinner, router)+0.01, 
                height,
                rpart,
                npart);
    
    //base plane
    if(ui->checkBox_vis_plane->isChecked())
    {
        Qt3DCore::QEntity* entity = new Qt3DCore::QEntity(scene);
        elements.emplace_back(entity);
        
        auto* mesh = new Qt3DExtras::QPlaneMesh;
        mesh->setHeight(2);
        mesh->setWidth(2);
        mesh->setMirrored(false);
        entity->addComponent(mesh);
        Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
        
        entity->addComponent(transform);
        entity->addComponent(material_other);
    }
    //base plane
    if(ui->checkBox_vis_plane->isChecked())
    {
        Qt3DCore::QEntity* entity = new Qt3DCore::QEntity(scene);
        elements.emplace_back(entity);
        
        auto* mesh = new Qt3DExtras::QPlaneMesh;
        mesh->setHeight(2);
        mesh->setWidth(2);
        mesh->setMirrored(true);
        entity->addComponent(mesh);
        Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
        
        entity->addComponent(transform);
        entity->addComponent(material_other);
    }
    //cyl outer
    if(ui->checkBox_vis_outer->isChecked())
    {
        Qt3DCore::QEntity* entity = new Qt3DCore::QEntity(scene);
        elements.emplace_back(entity);
        
        auto* mesh = new Qt3DExtras::QCylinderMesh;
        mesh->setSlices(20);
        mesh->setRings(3);
        mesh->setRadius(router);
        mesh->setLength(height);
        entity->addComponent(mesh);
        Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
        transform->setTranslation({0,static_cast<float>(height/2),0});
        
        entity->addComponent(transform);
        entity->addComponent(material_other);
    }
    //cyl inner
    if(ui->checkBox_vis_inner->isChecked())
    {
        Qt3DCore::QEntity* entity = new Qt3DCore::QEntity(scene);
        elements.emplace_back(entity);
        
        auto* mesh = new Qt3DExtras::QCylinderMesh;
        mesh->setSlices(20);
        mesh->setRings(3);
        mesh->setRadius(rinner);
        mesh->setLength(height);
        entity->addComponent(mesh);
        Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
        transform->setTranslation({0,static_cast<float>(height/2),0});
        
        entity->addComponent(transform);
        entity->addComponent(material_other);
    }
    //spheres
    for(std::size_t i = 0; i < npart;++i)
    {
        Qt3DCore::QEntity* sphere = new Qt3DCore::QEntity(scene);
        elements.emplace_back(sphere);
        
        Qt3DExtras::QSphereMesh* mesh = new Qt3DExtras::QSphereMesh;
        mesh->setRings(20);
        mesh->setSlices(20);
        mesh->setRadius(rpart);
        sphere->addComponent(mesh);
        Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
        transforms.emplace_back(transform);
        
        sphere->addComponent(transform);
        sphere->addComponent(material_sphere);
    }
}
