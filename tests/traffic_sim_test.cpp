#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <vector>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>

#include "traffic/traffic_sim.hpp"

TEST_CASE("brakeProb = 0 reaches free flow", "[sim][freeflow]") {
    constexpr int vMax = 5;
    constexpr int spacing = vMax + 2; // vMax + 1 empty cells between cars.
    constexpr int numCars = 12;
    constexpr int length = spacing * numCars;
    constexpr double density = static_cast<double>(numCars) / length;

    STATIC_REQUIRE(density < 1.0 / (vMax + 1));

    SECTION("evenly spaced cars all drive at vMax after vMax steps") {
        std::vector<int> positions;
        for (int i = 0; i < numCars; ++i) {
            positions.push_back(i * spacing);
        }

        traffic::TrafficSim sim({.length = length, .density = density, .vMax = vMax, .brakeProb = 0.0, .seed = 1});
        sim.setup(positions);
        REQUIRE(sim.numCars() == numCars);

        REQUIRE(sim.checkCount());
        for (int t = 0; t < vMax; ++t) {
            sim.step();
            CHECK(sim.checkCount());
        }

        const auto road = sim.road();
        for (std::size_t cell = 0; cell < road.size(); ++cell) {
            if (road[cell] >= 0) {
                INFO("cell " << cell << " has velocity " << road[cell]);
                CHECK(road[cell] == vMax);
            }
        }
    }

    SECTION("randomly placed cars reach vMax within a bounded transient") {
        traffic::TrafficSim sim(
            {.length = 1000, .density = 1.0 / (vMax + 2), .vMax = vMax, .brakeProb = 0.0, .seed = 1});
        sim.setup();

        constexpr int maxSteps = 5'000;
        int steps = 0;
        bool freeFlow = false;
        while (!freeFlow && steps < maxSteps) {
            sim.step();
            ++steps;
            CHECK(sim.checkCount());
            freeFlow = std::ranges::all_of(sim.road(), [](const int cell) { return cell < 0 || cell == vMax; });
        }

        INFO("steps to free flow: " << steps);
        CHECK(freeFlow);
    }
}

TEST_CASE("parameter limits", "[params]") {

    // rejected parameter both by the value and by the sim
    auto expectInvalidParams = [](const traffic::SimParams &params) {
        CHECK_THROWS_AS(params.validate(), std::invalid_argument);
        CHECK_THROWS_AS((traffic::TrafficSim{params}), std::invalid_argument);
    };

    SECTION("boundary values are accepted") {
        const traffic::TrafficSim no_cars({.length = 1, .density = 0.0, .vMax = 0, .brakeProb = 0.0});
        CHECK(no_cars.numCars() == 0);

        const traffic::TrafficSim full({.length = 10, .density = 1.0, .vMax = 5, .brakeProb = 1.0});
        CHECK(full.numCars() == 10);
    }

    SECTION("length must be positive") {
        expectInvalidParams({.length = 0});
        expectInvalidParams({.length = -1});
    }

    SECTION("density must be finite and within [0, 1]") {
        expectInvalidParams({.density = -0.5});
        expectInvalidParams({.density = 1.5});
        expectInvalidParams({.density = std::nan("")});
        expectInvalidParams({.density = std::numeric_limits<double>::infinity()});
    }

    SECTION("vMax must be non-negative") { //
        expectInvalidParams({.vMax = -1});
    }

    SECTION("brakeProb must be finite and within [0, 1]") {
        expectInvalidParams({.brakeProb = -0.1});
        expectInvalidParams({.brakeProb = 1.1});
        expectInvalidParams({.brakeProb = std::nan("")});
    }
}

TEST_CASE("Benchmark Long Road", "[sim][!benchmark]") {
    traffic::TrafficSim sim({.length = 10'000, .density = 0.3, .vMax = 5, .brakeProb = 0.15});
    sim.setup();

    BENCHMARK("Timesteps 5000") {
        for (int i = 0; i < 5'000; ++i) {
            sim.step();
        };
    };
}