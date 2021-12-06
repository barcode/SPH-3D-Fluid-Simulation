#include "cylindrical_wall_simulation_webserver.h"

int main(int argc, char* argv[])
{
    // Check command line arguments.
    if(argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <address> <port>\n";
        std::cerr << "  For IPv4, try:\n";
        std::cerr << "    receiver 0.0.0.0 80\n";
        std::cerr << "  For IPv6, try:\n";
        std::cerr << "    receiver 0::0 80\n";
        return EXIT_FAILURE;
    }
    cylindrical_wall_simulation_webserver webserver
    {
        argv[1], static_cast<unsigned short>(std::atoi(argv[2]))
    };
    webserver.wait();
    return 0;
}
