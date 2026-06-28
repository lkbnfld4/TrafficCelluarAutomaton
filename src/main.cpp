#include "traffic/traffic_sim.hpp"

#include <argparse/argparse.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("traffic");

    // clang-format off
    program.add_argument("length")
    .help("N, the length of the road")
    .scan<'d', int>();

    program.add_argument("density")
    .help("rho, the car density")
    .scan<'g', double>();

    program.add_argument("vMax")
    .help("v_{max}, the maximum velocity of car")
    .scan<'d', int>();

    program.add_argument("brakeProb")
    .help("p, the probability of a car braking")
    .scan<'g', double>();

    program.add_argument("steps")
    .help("T, the number of timesteps")
    .scan<'d', int>();

    program.add_argument("--seed")
    .help("seed for rng; 0 draws a random seed")
    .default_value(0)
    .scan<'d', int>();

    // clang-format on

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    const int steps = program.get<int>("steps");

    try {
        traffic::TrafficSim sim({.length = program.get<int>("length"),
                                 .density = program.get<double>("density"),
                                 .vMax = program.get<int>("vMax"),
                                 .brakeProb = program.get<double>("brakeProb"),
                                 .seed = program.get<int>("seed")});
        sim.setup();

        for (int i = 0; i < steps; ++i) {
            sim.step();
            sim.print(std::cout);
        }
    } catch (const std::invalid_argument &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    return 0;
}
