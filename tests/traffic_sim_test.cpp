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

namespace {

using Road = std::vector<std::vector<int>>;

// deterministic: no random braking, fixed seed.
traffic::SimParams deterministicParams(const int lanes,    //
                                       const int length,   //
                                       const int cars,     //
                                       const int vMax = 5, //
                                       const int lookahead = 16) {
    return {.lanes = lanes,
            .length = length,
            .density = static_cast<double>(cars) / (lanes * length),
            .vMax = vMax,
            .brakeProb = 0.0,
            .laneChangeLookahead = lookahead,
            .seed = 1};
}

Road emptyRoad(const int lanes, const int length) { //
    return Road(lanes, std::vector<int>(length, -1));
}

} // namespace

TEST_CASE("brakeProb = 0 reaches free flow", "[sim][freeflow]") {
    constexpr int vMax = 5;
    constexpr int spacing = vMax + 2; // vMax + 1 empty cells between cars.
    constexpr int numCars = 12;
    constexpr int lanes = 1;
    constexpr int length = spacing * numCars;
    constexpr double density = static_cast<double>(numCars) / length;

    STATIC_REQUIRE(density < 1.0 / (vMax + 1));

    SECTION("evenly spaced cars all drive at vMax after vMax steps") {
        std::vector<std::vector<int>> positions(1);
        for (int i = 0; i < numCars; ++i) {
            positions[0].push_back(i * spacing);
        }

        traffic::TrafficSim sim({.lanes = lanes,     //
                                 .length = length,   //
                                 .density = density, //
                                 .vMax = vMax,       //
                                 .brakeProb = 0.0,   //
                                 .seed = 1});
        sim.setup(positions);
        REQUIRE(sim.numCars() == numCars);

        REQUIRE(sim.checkCount());
        for (int t = 0; t < vMax; ++t) {
            sim.step();
            CHECK(sim.checkCount());
        }

        const auto lane = sim.road()[0];
        for (std::size_t cell = 0; cell < lane.size(); ++cell) {
            if (lane[cell] >= 0) {
                INFO("cell " << cell << " has velocity " << lane[cell]);
                CHECK(lane[cell] == vMax);
            }
        }
    }

    SECTION("randomly placed cars reach vMax within a bounded transient") {
        traffic::TrafficSim sim({.lanes = 1,                  //
                                 .length = 1000,              //
                                 .density = 1.0 / (vMax + 2), //
                                 .vMax = vMax,                //
                                 .brakeProb = 0.0,            //
                                 .seed = 1});
        sim.setup();

        constexpr int maxSteps = 5'000;
        int steps = 0;
        bool freeFlow = false;
        while (!freeFlow && steps < maxSteps) {
            sim.step();
            ++steps;
            CHECK(sim.checkCount());
            freeFlow = std::ranges::all_of(sim.road()[0], [](const int cell) { return cell < 0 || cell == vMax; });
        }

        INFO("steps to free flow: " << steps);
        CHECK(freeFlow);
    }
}

TEST_CASE("random setup places every car", "[sim][setup]") {
    SECTION("car count that is not a multiple of the lane count") {
        traffic::TrafficSim sim(deterministicParams(2, 20, 7));
        REQUIRE(sim.numCars() == 7);

        CHECK_NOTHROW(sim.setup());
        CHECK(sim.checkCount());
    }

    SECTION("more lanes than cars") {
        traffic::TrafficSim sim(deterministicParams(4, 10, 2));
        REQUIRE(sim.numCars() == 2);

        CHECK_NOTHROW(sim.setup());
        CHECK(sim.checkCount());
    }

    SECTION("full occupancy") {
        traffic::TrafficSim sim(deterministicParams(3, 10, 30));
        REQUIRE(sim.numCars() == 30);

        CHECK_NOTHROW(sim.setup());
        CHECK(sim.checkCount());
        for (const std::vector<int> &lane : sim.road()) {
            CHECK(std::ranges::all_of(lane, [](const int cell) { return cell == 0; }));
        }
    }
}

TEST_CASE("setup validates before changing the road", "[sim][setup]") {
    traffic::TrafficSim sim(deterministicParams(2, 20, 4));
    REQUIRE_NOTHROW(sim.setup({{1, 5}, {10, 15}}));

    CHECK_THROWS_AS(sim.setup({{-1, 5}, {10, 15}}), std::invalid_argument); // position before the start
    CHECK_THROWS_AS(sim.setup({{1, 20}, {10, 15}}), std::invalid_argument); // position at the end
    CHECK_THROWS_AS(sim.setup({{1, 1}, {10, 15}}), std::invalid_argument);  // duplicate within a lane
    CHECK_THROWS_AS(sim.setup({{1, 5}, {10}}), std::invalid_argument);      // wrong car count
    CHECK_THROWS_AS(sim.setup({{1, 5}}), std::invalid_argument);            // wrong lane count
}

TEST_CASE("lane changes preserves car", "[sim][lanechange]") {
    SECTION("left to right") {
        traffic::TrafficSim sim(deterministicParams(2, 40, 2));
        REQUIRE_NOTHROW(sim.setup({{5}, {10}}));
        REQUIRE(sim.checkCount());

        sim.step();

        CHECK(sim.checkCount());

        Road expected = emptyRoad(2, 40);
        expected[1][6] = 1;
        expected[1][11] = 1;
        CHECK(sim.road() == expected);
    }

    SECTION("right to left") {
        traffic::TrafficSim sim(deterministicParams(2, 40, 1));
        REQUIRE_NOTHROW(sim.setup({{}, {5}}));
        sim.step(); // left to right

        Road afterFirst = emptyRoad(2, 40);
        afterFirst[1][6] = 1;
        REQUIRE(sim.road() == afterFirst);

        sim.step();

        CHECK(sim.checkCount());

        Road expected = emptyRoad(2, 40);
        expected[0][8] = 2;
        CHECK(sim.road() == expected);
    }
}

TEST_CASE("traffic metrics", "[sim][metrics]") {
    traffic::TrafficSim sim(deterministicParams(3, 32, 3, 5, 5));
    const Road positions = {{5}, {10}, {15}};
    sim.setup(positions);
    CHECK(sim.laneUsage(0) == 1.0 / 3);
    CHECK(sim.laneUsage(1) == 1.0 / 3);
    CHECK(sim.laneUsage(2) == 1.0 / 3);
    CHECK(sim.flowrate() == 0.0);

    sim.step(); // 2 left to right changes
    CHECK(sim.laneUsage(0) == 0.0);
    CHECK(sim.laneUsage(1) == 1.0 / 3);
    CHECK(sim.laneUsage(2) == 2.0 / 3);
    CHECK(sim.flowrate(0) == 0.0);
    CHECK(sim.flowrate(1) == 1.0 / 32);
    CHECK(sim.flowrate(2) == 2.0 / 32);
    CHECK(sim.flowrate() == 3.0 / 32);
    CHECK(sim.laneChangeFrequency() == 2.0 / 3);

    sim.step(); // 2 right to left changes
    CHECK(sim.laneUsage(0) == 1.0 / 3);
    CHECK(sim.laneUsage(1) == 1.0 / 3);
    CHECK(sim.laneUsage(2) == 1.0 / 3);
    CHECK(sim.flowrate(0) == 2.0 / 32);
    CHECK(sim.flowrate(1) == 2.0 / 32);
    CHECK(sim.flowrate(2) == 2.0 / 32);
    CHECK(sim.flowrate() == 6.0 / 32);
    CHECK(sim.laneChangeFrequency() == 2.0 / 3);
}

TEST_CASE("blocked inside lane keeps the car in its lane", "[sim][lanechange]") {
    traffic::TrafficSim sim(deterministicParams(3, 40, 3));
    REQUIRE_NOTHROW(sim.setup({{9}, {7}, {24}}));
    REQUIRE(sim.checkCount());

    sim.step();

    CHECK(sim.checkCount());

    Road expected = emptyRoad(3, 40);
    expected[0][10] = 1;
    expected[1][8] = 1;
    expected[2][25] = 1;
    CHECK(sim.road() == expected);
}

TEST_CASE("the lane-change lookahead", "[sim][lanechange]") {
    const std::vector<std::vector<int>> positions = {{0, 5}, {20}};

    SECTION("a leader beyond the lookahead is ignored") {
        traffic::TrafficSim sim(deterministicParams(2, 40, 3, 5, 4));
        REQUIRE_NOTHROW(sim.setup(positions));

        sim.step();

        Road expected = emptyRoad(2, 40);
        expected[0][1] = 1;
        expected[0][6] = 1;
        expected[1][21] = 1;
        CHECK(sim.road() == expected);
    }

    SECTION("a leader at the lookahead is seen") {
        traffic::TrafficSim sim(deterministicParams(2, 40, 3, 5, 5));
        REQUIRE_NOTHROW(sim.setup(positions));

        sim.step();

        Road expected = emptyRoad(2, 40);
        expected[0][6] = 1;
        expected[1][1] = 1;
        expected[1][21] = 1;
        CHECK(sim.road() == expected);
    }
}

TEST_CASE("empty road averages zero", "[sim]") {
    traffic::TrafficSim sim(deterministicParams(3, 10, 0));
    REQUIRE_NOTHROW(sim.setup());
    REQUIRE(sim.numCars() == 0);

    for (int t = 0; t < 3; ++t) {
        CHECK_NOTHROW(sim.step());
        CHECK(sim.checkCount());
    }

    CHECK(sim.avgVelocity() == 0.0);
    CHECK(sim.avgVelocity(0) == 0.0);
    CHECK(sim.avgVelocity(1) == 0.0);
    CHECK(sim.avgVelocity(2) == 0.0);
    CHECK_THROWS_AS(sim.avgVelocity(-1), std::out_of_range);
    CHECK_THROWS_AS(sim.avgVelocity(3), std::out_of_range);
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

    SECTION("laneChangeLookahead must be positive") {
        expectInvalidParams({.laneChangeLookahead = 0});
        expectInvalidParams({.laneChangeLookahead = -1});
    }
}

TEST_CASE("Benchmark Long Road", "[sim][!benchmark]") {
    traffic::TrafficSim sim({.length = 10'000, .density = 0.3, .vMax = 5, .brakeProb = 0.25});
    sim.setup();

    BENCHMARK("Timesteps 5000") {
        for (int i = 0; i < 5'000; ++i) {
            sim.step();
        };
    };
}