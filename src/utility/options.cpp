#include <utility/options.h>
#include <iostream>

using namespace cauv;

bool cauv::Options::parseOptions(int argc, char **argv) {
    namespace po = boost::program_options;
    desc.add_options()
        ("help,h", "Print this help message")
        ("version,V", "show version information")
    ;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
    if(vm.count("help"))
    {
        std::cerr << desc << std::flush;
        return true;
    }
    if(vm.count("version"))
    {
        //std::cout << Version_Information << std::flush;
        return true;
    }
    po::notify(vm);
    return false;
}
