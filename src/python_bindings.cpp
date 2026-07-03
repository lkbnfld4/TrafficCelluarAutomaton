#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "traffic/traffic_sim.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <sstream>

namespace nb = nanobind;
using namespace nb::literals;
using traffic::SimParams;
using traffic::TrafficSim;
using Positions = nb::ndarray<nb::numpy, const std::int64_t, nb::shape<-1, 2>, nb::device::cpu>;

NB_MODULE(core, m) {
    constexpr SimParams defaults{};
    nb::class_<SimParams>(m, "SimParams")
        .def(nb::init<int, int, double, int, double, int, int, int>(),
             "lanes"_a = defaults.lanes,     //
             "length"_a = defaults.length,   //
             "density"_a = defaults.density, //
             "vMax"_a = defaults.vMax,
             "brakeProb"_a = defaults.brakeProb,                     //
             "laneChangeLookahead"_a = defaults.laneChangeLookahead, //
             "slack"_a = defaults.slack, "seed"_a = defaults.seed)
        .def_rw("lanes", &SimParams::lanes)
        .def_rw("length", &SimParams::length)
        .def_rw("density", &SimParams::density)
        .def_rw("vMax", &SimParams::vMax)
        .def_rw("brakeProb", &SimParams::brakeProb)
        .def_rw("laneChangeLookahead", &SimParams::laneChangeLookahead)
        .def_rw("slack", &SimParams::slack)
        .def_rw("seed", &SimParams::seed)
        .def("validate", &SimParams::validate);

    nb::class_<TrafficSim>(m, "TrafficSim")
        .def(nb::init<const SimParams &>(), "params"_a)
        .def_prop_ro(
            "params_", [](const TrafficSim &sim) { return sim.params_; }, "A copy of the simulator parameters.")
        .def("setup", nb::overload_cast<>(&TrafficSim::setup))
        .def(
            "setup",
            [](TrafficSim &sim, Positions positions) {
                std::vector<std::vector<int>> lanes(sim.params_.lanes);
                for (int i = 0; i < positions.shape(0); ++i) {
                    const auto lane = positions(i, 0);
                    const auto cell = positions(i, 1);
                    lanes[lane].push_back(static_cast<int>(cell));
                }
                sim.setup(lanes);
            },
            "positions"_a.noconvert(), "Place cars from an int64 NumPy array of (lane, cell) pairs.")
        .def("step", &TrafficSim::step)
        .def(
            "road",
            [](const TrafficSim &sim) {
                const auto &road = sim.road();
                const auto length = road.front().size();

                auto buffer = std::make_unique_for_overwrite<int[]>(road.size() * length);
                auto *data = buffer.get();
                nb::capsule owner(data, [](void *ptr) noexcept { delete[] static_cast<int *>(ptr); });
                buffer.release();
                for (int lane = 0; lane < road.size(); ++lane) {
                    std::ranges::copy(road[lane], data + lane * length);
                }
                return nb::ndarray<nb::numpy, int, nb::ndim<2>>(data, {road.size(), length}, owner);
            },
            "Return a NumPy snapshot of shape (lanes, length), with -1 for empty cells.")
        .def("numCars", &TrafficSim::numCars)
        .def("checkCount", &TrafficSim::checkCount)
        .def("avgVelocity", nb::overload_cast<>(&TrafficSim::avgVelocity, nb::const_))
        .def("avgVelocity", nb::overload_cast<int>(&TrafficSim::avgVelocity, nb::const_), "lane"_a)
        .def("laneUsage", &TrafficSim::laneUsage, "lane"_a)
        .def("flowrate", nb::overload_cast<>(&TrafficSim::flowrate, nb::const_))
        .def("flowrate", nb::overload_cast<int>(&TrafficSim::flowrate, nb::const_), "lane"_a)
        .def("laneChangeFrequency", &TrafficSim::laneChangeFrequency)
        .def("print", [](const TrafficSim &sim) {
            std::ostringstream out;
            sim.print(out);
            const auto text = out.view();
            nb::print(nb::str(text.data(), text.size()), nb::str(""));
        });
}
