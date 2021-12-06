#include <stdexcept>
#include <random>
#include <cstring>
#include <chrono>
#include <type_traits>
#include <atomic>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <regex>
#include <thread>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio.hpp>
#include <boost/dll.hpp>
#include <boost/gil.hpp>
#include <boost/gil/extension/io/jpeg.hpp>

#include "cylindrical_wall_simulation_webserver.h"
#include "PARTICLE_SYSTEM.h"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

//cylindrical_wall_simulation_webserver::sim_output&
//cylindrical_wall_simulation_webserver::output_write_buffer()
//{
//    return output.getWriteBuffer();
//}
std::string readfile(const std::string& name)
{
    std::ifstream ifs{boost::dll::program_location().parent_path() / name};
    return {(std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>())};
}

cylindrical_wall_simulation_webserver::sim_output::out_lights
cylindrical_wall_simulation_webserver::lights() const
{
    sim_output::out_lights l;
    {
        std::lock_guard g{output_read_mutex};
        l = output.getUpToDateReadBuffer().lights;
    }
    return l;
}
std::vector<VEC3D> cylindrical_wall_simulation_webserver::centers() const
{
    std::vector<VEC3D> c;
    {
        std::lock_guard g{output_read_mutex};
        c = output.getUpToDateReadBuffer().centers;
    }
    return c;
}

cylindrical_wall_simulation_webserver::~cylindrical_wall_simulation_webserver()
{
    stop_thread_sim = true;
    stop_thread_web = true;
    wait();
}

cylindrical_wall_simulation_webserver::cylindrical_wall_simulation_webserver(
        const std::string& ip,
        unsigned short port) :
    ip{ip}, port{port}
{
    thread_sim = std::thread{[&]{task_sim();}};
    thread_web = std::thread{[&]{task_web();}};
}

void cylindrical_wall_simulation_webserver::task_sim()
{
    cylindrical_wall_simulation sim(
                sim_state.setup_radius_inner ,
                sim_state.setup_radius_outer ,
                sim_state.setup_height       ,
                sim_state.setup_radius_particles,
                sim_state.setup_num_particles
    );

    diode_grid d_grid;

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
        if(sim_state.setup_reset_sim.exchange(false))
        {
            const auto rinner = sim_state.setup_radius_inner .load();
            const auto router = sim_state.setup_radius_outer .load();
            sim.reset(std::min(rinner, router),
                      std::max(rinner, router)+0.01,
                      sim_state.setup_height       ,
                      sim_state.setup_radius_particles,
                      sim_state.setup_num_particles
            );
        }

        sim.gravity(sim_state.sim_gravity_x         ,
                     sim_state.sim_gravity_y         ,
                     sim_state.sim_gravity_z         );

        sim._catch_escaped_particles = sim_state.sim_catch_particles   ;

        d_grid.height = sim_state.setup_height       ;

        sim._particle_system->GAS_STIFFNESS     = sim_state.sim_GAS_STIFFNESS     ;
        sim._particle_system->REST_DENSITY      = sim_state.sim_REST_DENSITY      ;
        sim._particle_system->PARTICLE_MASS     = sim_state.sim_PARTICLE_MASS     ;
        sim._particle_system->VISCOSITY         = sim_state.sim_VISCOSITY         ;
        sim._particle_system->SURFACE_THRESHOLD = sim_state.sim_SURFACE_THRESHOLD ;
        sim._particle_system->SURFACE_TENSION   = sim_state.sim_SURFACE_TENSION   ;
        sim._particle_system->KERNEL_PARTICLES  = sim_state.sim_KERNEL_PARTICLES  ;
        sim._particle_system->WALL_K            = sim_state.sim_WALL_K            ;
        sim._particle_system->WALL_DAMPING      = sim_state.sim_WALL_DAMPING      ;

        d_grid.n_height      = sim_state.num_diodes_height     ;
        d_grid.n_circ        = sim_state.num_diodes_width      ;
        d_grid.brightness    = sim_state.brightness            ;
        d_grid.n_radius_circ = sim_state.light_radius_height   ;
        d_grid.n_radius_h    = sim_state.light_radius_width    ;

        if(sim_state.sim_zero_vel.exchange(false))
        {
            sim.visit_particles_mod([&](int idx, auto& part)
            {
                part.velocity() = VEC3D{0,0,0};
            });
        }
        if(sim_state.sim_dt_per_step > 0)
        {
            sim.step(sim_state.sim_dt_per_step);
        }
        auto& wb = output.getWriteBuffer();
        wb.centers.resize(sim._particle_system->num_particles());
        d_grid.clear_light();
        sim.visit_particles([&](int idx, const auto& part)
        {
            const auto& p = part.position();
            if(idx < wb.centers.size())
            {
                wb.centers.at(idx) = p;
            }
            else
            {
                std::cout << "IDX " << idx << " >= " << wb.centers.size() << '\n';
            }
            try
            {
                d_grid.add_light(p.x,p.y,p.z);
            }
            catch(...)
            {
                std::cout << "failed to add light "
                << p.x << " / " << p.y << " / "<< p.z << "\n";
            }
        });
        wb.lights.lights   = d_grid.diodes;
        wb.lights.lights_w = d_grid.n_circ;
        wb.lights.lights_h = d_grid.n_height;
        output.commitWrite();
    }
}


void cylindrical_wall_simulation_webserver::wait()
{
    if(thread_sim.joinable())
    {
        thread_sim.join();
    }
    if(thread_web.joinable())
    {
        thread_web.join();
    }
}


class http_connection : public std::enable_shared_from_this<http_connection>
{
public:
    template<class T>
    void set(const std::string& val, std::atomic<T>& targ)
    {
        if constexpr(std::is_same_v<T, bool>)
        {
            targ = (val == "true");
        }
        else if constexpr(std::is_same_v<T, std::size_t> || std::is_same_v<T, double>)
        {
            std::stringstream str;
            str << val;
            T res;
            str>>res;
            targ = res;
        }
        else
        {
            static_assert(!std::is_same_v<T, T>);
        }
        response_.set(http::field::content_type, "text/plain");
        beast::ostream(response_.body()) << "updated parameter to " << targ.load();
    }

    http_connection(
            tcp::socket socket,
            cylindrical_wall_simulation_webserver* server,
            cylindrical_wall_simulation_webserver::simulator_state* state)
        : socket_(std::move(socket)), sim_server(server), sim_state(state)
    {
    }

    // Initiate the asynchronous operations associated with the connection.
    void
    start()
    {
        read_request();
        check_deadline();
    }

private:
    cylindrical_wall_simulation_webserver::simulator_state* sim_state = nullptr;
    cylindrical_wall_simulation_webserver* sim_server = nullptr;
    // The socket for the currently connected client.
    tcp::socket socket_;

    // The buffer for performing reads.
    beast::flat_buffer buffer_{8192};

    // The request message.
    http::request<http::dynamic_body> request_;

    // The response message.
    http::response<http::dynamic_body> response_;
    http::response<http::file_body> response_file;
    bool use_response_file = false;

    // The timer for putting a deadline on connection processing.
    net::steady_timer deadline_{
        socket_.get_executor(), std::chrono::seconds(60)};

    // Asynchronously receive a complete request message.
    void
    read_request()
    {
        auto self = shared_from_this();

        http::async_read(
            socket_,
            buffer_,
            request_,
            [self](beast::error_code ec,
                std::size_t bytes_transferred)
            {
                boost::ignore_unused(bytes_transferred);
                if(!ec)
                    self->process_request();
            });
    }

    // Determine what needs to be done with the request message.
    void
    process_request()
    {
        response_.version(request_.version());
        response_.keep_alive(false);

        response_file.version(request_.version());
        response_file.keep_alive(false);

        use_response_file = false;

        switch(request_.method())
        {
        case http::verb::get:
            response_.result(http::status::ok);
            response_.set(http::field::server, "Beast");
            response_file.result(http::status::ok);
            response_file.set(http::field::server, "Beast");
            create_response();
            break;

        default:
            // We return responses indicating an error if
            // we do not recognize the request method.
            response_.result(http::status::bad_request);
            response_.set(http::field::content_type, "text/plain");
            beast::ostream(response_.body())
                << "Invalid request-method '"
                << std::string(request_.method_string())
                << "'";
            break;
        }

        write_response();
    }

    // Construct a response message based on the program state.
    void
    create_response()
    {

        static const std::regex regex{R"(/([^?]*)(\?val=(.*))?)"};
        std::smatch match;
        const auto full_targ_sv = request_.target();
        const std::string full_targ(full_targ_sv.data(), full_targ_sv.size());
        std::regex_match(full_targ,match,regex);
        const auto targ = match.str(1);
        const auto val  = match.str(3);

        std::cout << request_.target() << " -> " << targ << " (" << val << ")\n";

        if(targ == "time")
        {
            response_.set(http::field::content_type, "text/html");
            beast::ostream(response_.body())
                <<  "<html>\n"
                <<  "<head><title>Current time</title></head>\n"
                <<  "<body>\n"
                <<  "<h1>Current time</h1>\n"
                <<  "<p>The current time is "
                <<  sim_state->now()
                <<  " nano seconds since the epoch.</p>\n"
                <<  "</body>\n"
                <<  "</html>\n";
        }
        else if(targ == "quit")
        {
            response_.set(http::field::content_type, "text/html");
            beast::ostream(response_.body())
                <<  "<html>\n"
                <<  "<head><title>shutting down</title></head>\n"
                <<  "<body>\n"
                <<  "<h1>shutting down</h1>\n"
                <<  "</body>\n"
                <<  "</html>\n";
            std::cout << "shutdown was called!\n";
            write_response();
            std::exit(0);
        }
        else if(targ == "configure")
        {
            response_.set(http::field::content_type, "text/html");

            beast::ostream(response_.body()) << readfile("configure.html");
        }
        else if(targ == "sup_reset" ) {sim_state->setup_reset_sim = true;}
        else if(targ == "sim_zero_vel") {sim_state->sim_zero_vel = true;}
        else if(targ == "sup_ri"    ) {set(val, sim_state->setup_radius_inner    );}
        else if(targ == "sup_ro"    ) {set(val, sim_state->setup_radius_outer    );}
        else if(targ == "sup_h"     ) {set(val, sim_state->setup_height          );}
        else if(targ == "sup_np"    ) {set(val, sim_state->setup_num_particles   );}
        else if(targ == "sup_rp"    ) {set(val, sim_state->setup_radius_particles);}
        else if(targ == "sim_gx"    ) {set(val, sim_state->sim_gravity_x         );}
        else if(targ == "sim_gy"    ) {set(val, sim_state->sim_gravity_y         );}
        else if(targ == "sim_gz"    ) {set(val, sim_state->sim_gravity_z         );}
        else if(targ == "sim_dt"    ) {set(val, sim_state->sim_dt_per_step       );}
        else if(targ == "sim_gs"    ) {set(val, sim_state->sim_GAS_STIFFNESS     );}
        else if(targ == "sim_rd"    ) {set(val, sim_state->sim_REST_DENSITY      );}
        else if(targ == "sim_pm"    ) {set(val, sim_state->sim_PARTICLE_MASS     );}
        else if(targ == "sim_vs"    ) {set(val, sim_state->sim_VISCOSITY         );}
        else if(targ == "sim_str"   ) {set(val, sim_state->sim_SURFACE_THRESHOLD );}
        else if(targ == "sim_ste"   ) {set(val, sim_state->sim_SURFACE_TENSION   );}
        else if(targ == "sim_kp"    ) {set(val, sim_state->sim_KERNEL_PARTICLES  );}
        else if(targ == "sim_wk"    ) {set(val, sim_state->sim_WALL_K            );}
        else if(targ == "sim_wd"    ) {set(val, sim_state->sim_WALL_DAMPING      );}
        else if(targ == "sim_cat"   ) {set(val, sim_state->sim_catch_particles   );}
        else if(targ == "d_d"       ) {set(val, sim_state->brightness            );}
        else if(targ == "d_nh"      ) {set(val, sim_state->num_diodes_height     );}
        else if(targ == "d_nw"      ) {set(val, sim_state->num_diodes_width      );}
        else if(targ == "d_rh"      ) {set(val, sim_state->light_radius_height   );}
        else if(targ == "d_rw"      ) {set(val, sim_state->light_radius_width    );}
        else if(targ == "d_b2p"     ) {set(val, sim_state->d_brigh_to_pxval      );}
        else if(targ == "params"    )
        {
            response_.set(http::field::content_type, "text/plain");
            beast::ostream(response_.body())
            << "{"
            << R"(      "sup_ri"    : )" << sim_state->setup_radius_inner    .load()
            << R"(    , "sup_ro"    : )" << sim_state->setup_radius_outer    .load()
            << R"(    , "sup_h"     : )" << sim_state->setup_height          .load()
            << R"(    , "sup_np"    : )" << sim_state->setup_num_particles   .load()
            << R"(    , "sup_rp"    : )" << sim_state->setup_radius_particles.load()
            << R"(    , "sim_zero_vel" : )" << sim_state->sim_zero_vel         .load()
            << R"(    , "sim_gx"    : )" << sim_state->sim_gravity_x         .load()
            << R"(    , "sim_gy"    : )" << sim_state->sim_gravity_y         .load()
            << R"(    , "sim_gz"    : )" << sim_state->sim_gravity_z         .load()
            << R"(    , "sim_dt"    : )" << sim_state->sim_dt_per_step       .load()
            << R"(    , "sim_gs"    : )" << sim_state->sim_GAS_STIFFNESS     .load()
            << R"(    , "sim_rd"    : )" << sim_state->sim_REST_DENSITY      .load()
            << R"(    , "sim_pm"    : )" << sim_state->sim_PARTICLE_MASS     .load()
            << R"(    , "sim_vs"    : )" << sim_state->sim_VISCOSITY         .load()
            << R"(    , "sim_str"   : )" << sim_state->sim_SURFACE_THRESHOLD .load()
            << R"(    , "sim_ste"   : )" << sim_state->sim_SURFACE_TENSION   .load()
            << R"(    , "sim_kp"    : )" << sim_state->sim_KERNEL_PARTICLES  .load()
            << R"(    , "sim_wk"    : )" << sim_state->sim_WALL_K            .load()
            << R"(    , "sim_wd"    : )" << sim_state->sim_WALL_DAMPING      .load()
            << R"(    , "sim_cat"   : )" << sim_state->sim_catch_particles   .load()
            << R"(    , "d_d"       : )" << sim_state->brightness            .load()
            << R"(    , "d_nh"      : )" << sim_state->num_diodes_height     .load()
            << R"(    , "d_nw"      : )" << sim_state->num_diodes_width      .load()
            << R"(    , "d_rh"      : )" << sim_state->light_radius_height   .load()
            << R"(    , "d_rw"      : )" << sim_state->light_radius_width    .load()
            << R"(    , "d_b2p"     : )" << sim_state->d_brigh_to_pxval      .load()
            << R"(    , "time"      : )" << sim_state->now()
            << "}";
        }
        else if(targ == "diodes")
        {
            const auto l = sim_server->lights();
            if(l.lights.size() != l.lights_w*l.lights_h)
            {
                std::cout << "inconsistency in lights!"
                          << " w " << l.lights_w
                          << " h " << l.lights_h
                          << " # " << l.lights.size()
                          << " (shold have " << l.lights_w*l.lights_h << ")\n";
            }
            boost::gil::gray8_image_t image{
                static_cast<long int>(l.lights_w),
                static_cast<long int>(l.lights_h)
            };
            auto view = boost::gil::view(image);
            for(std::size_t i = 0; i < l.lights.size();++i)
            {
                const int x = i % l.lights_w;
                const int y = l.lights_h -1 - (i / l.lights_w);
                const std::uint8_t val = static_cast<std::uint8_t>(std::min(255.0, l.lights.at(i) / sim_state->d_brigh_to_pxval));
                if(x<l.lights_w && y<l.lights_h)
                {
                    view(x,y) = boost::gil::gray8_pixel_t{val};
                }
                else
                {
                    std::cout << "failed to set pixel "
                    << x << " / " << y << "\n";
                }
            }
            response_.set(http::field::content_type, "image/jpeg");

            {
                std::ofstream o("diodes2.jpeg", std::ios_base::binary );
                boost::gil::write_view(o, view, boost::gil::jpeg_tag());
            }


            use_response_file = true;
            http::file_body::value_type body;
            beast::error_code ec;
            response_file.body().open("diodes2.jpeg", beast::file_mode::scan, ec);
        }
        else if(targ == "diodes_val")
        {
            const auto l = sim_server->lights();
            response_.set(http::field::content_type, "text/plain");
            auto out = beast::ostream(response_.body());
            out << '[';
            bool first = true;
            for(const auto& v:l.lights)
            {
                if(!first)
                {
                    out << ',';
                }
                out << v;
                first = false;
            }
            out << ']';
        }
        else if(targ == "positions")
        {
            response_.set(http::field::content_type, "text/plain");
            auto out = beast::ostream(response_.body());
            const auto centers = sim_server->centers();
            out << '[';
            bool first = true;
            {
                for(const auto& p:centers)
                {
                    if(!first)
                    {
                        out << ',';
                    }
                    out << '[' << p.x << ','<< p.y << ','<< p.z << ']';
                    first = false;
                }
            }
            out << ']';

        }
        else
        {
            response_.result(http::status::not_found);
            response_.set(http::field::content_type, "text/plain");
            beast::ostream(response_.body()) << "File not found\r\n";
        }
    }

    // Asynchronously transmit the response message.
    void
    write_response()
    {
        auto self = shared_from_this();

        if(use_response_file)
        {
            response_file.content_length(response_file.body().size());

            http::async_write(
                socket_,
                response_file,
                [self](beast::error_code ec, std::size_t)
                {
                    self->socket_.shutdown(tcp::socket::shutdown_send, ec);
                    self->deadline_.cancel();
                });
        }
        else
        {
            response_.content_length(response_.body().size());

            http::async_write(
                socket_,
                response_,
                [self](beast::error_code ec, std::size_t)
                {
                    self->socket_.shutdown(tcp::socket::shutdown_send, ec);
                    self->deadline_.cancel();
                });
        }
    }

    // Check whether we have spent enough time on this connection.
    void
    check_deadline()
    {
        auto self = shared_from_this();

        deadline_.async_wait(
            [self](beast::error_code ec)
            {
                if(!ec)
                {
                    // Close socket to cancel any outstanding operation.
                    self->socket_.close(ec);
                }
            });
    }
};

void http_server(
        cylindrical_wall_simulation_webserver* server,
        cylindrical_wall_simulation_webserver::simulator_state* state,
        tcp::acceptor& acceptor,
        tcp::socket& socket)
{
    acceptor.async_accept(socket,
        [&, server, state](beast::error_code ec)
        {
            if(!ec)
                std::make_shared<http_connection>(std::move(socket), server, state)->start();
            http_server(server, state, acceptor, socket);
        });
}
void cylindrical_wall_simulation_webserver::task_web()
{
    auto const address = net::ip::make_address(ip);
    net::io_context ioc{1};
    tcp::acceptor acceptor{ioc, {address, port}};
    tcp::socket socket{ioc};
    http_server(this, &sim_state, acceptor, socket);

    ioc.run();
    while(!stop_thread_web)
    {
        ioc.run_one();
    }
}
