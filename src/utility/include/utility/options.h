#pragma once

#include <boost/program_options.hpp>

#include <string>

namespace cauv {

class Options {
    public:
    Options(std::string description) : desc(description) {};
    bool parseOptions(int argc, char **argv);
    boost::program_options::options_description desc;
    boost::program_options::positional_options_description pos;
    boost::program_options::variables_map vm;
};

};
