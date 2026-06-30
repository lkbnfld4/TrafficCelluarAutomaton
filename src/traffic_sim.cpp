#include "traffic/traffic_sim.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <ostream>
#include <ranges>
#include <stdexcept>

namespace traffic {

namespace {

// circular index
int wrapForward(const int pos, const int offset, const int length) { //
    return (pos + offset) % length;
}

int wrapBackward(const int pos, const int offset, const int length) { //
    return (pos - offset + length) % length;
}

} // namespace

SimParams SimParams::validate() const {
    if (lanes <= 0) {
        throw std::invalid_argument("lanes must be positive");
    }
    if (length <= 0) {
        throw std::invalid_argument("length must be positive");
    }
    // negated so NaN is rejected
    if (!(density >= 0.0 && density <= 1.0)) {
        throw std::invalid_argument("density must be in [0, 1]");
    }
    if (vMax < 0) {
        throw std::invalid_argument("vMax must be non-negative");
    }
    if (!(brakeProb >= 0.0 && brakeProb <= 1.0)) {
        throw std::invalid_argument("brakeProb must be in [0, 1]");
    }
    if (laneChangeLookahead <= 0) {
        throw std::invalid_argument("laneChangeLookahead must be positive");
    }
    if (slack < 0) {
        throw std::invalid_argument("slack must be non-negative");
    }
    return *this;
}

// clang-format off
TrafficSim::TrafficSim(const SimParams &params)
    : params_(params.validate()),
      road_(params.lanes, std::vector<int>(params.length, -1)),
      nextRoad_(params.lanes, std::vector<int>(params.length, -1)),
      numCars_(std::lround(params.lanes * params.length * params.density)),
      brakeDist_(params.brakeProb),
      rng_(params.seed == 0 ? std::random_device{}() : params.seed) {}
// clang-format on

void TrafficSim::setup() {
    const int totalCells = params_.lanes * params_.length;
    std::vector<int> start(numCars_);
    std::ranges::sample(std::views::iota(0, totalCells), start.begin(), numCars_, rng_);

    std::vector<std::vector<int>> positions(params_.lanes);
    for (const int cell : start) {
        positions[cell / params_.length].push_back(wrapForward(cell, 0, params_.length));
    }
    place(positions);
}

void TrafficSim::setup(const std::vector<std::vector<int>> &positions) { //
    place(positions);
}

void TrafficSim::place(const std::vector<std::vector<int>> &positions) {
    if (positions.size() != static_cast<std::size_t>(params_.lanes)) {
        throw std::invalid_argument("positions incorrect number of lanes");
    }

    int total = 0;
    for (const std::vector<int> &lane : positions) {
        total += lane.size();
    }
    if (total != numCars_) {
        throw std::invalid_argument("positions incorrect number of cars");
    }

    // build in nextRoad_ so if rejected leaves road_ untouched.
    for (int i = 0; i < params_.lanes; ++i) {
        std::ranges::fill(nextRoad_[i], -1);
        for (const int &pos : positions[i]) {
            if (pos < 0 || pos >= params_.length || nextRoad_[i][pos] >= 0) {
                throw std::invalid_argument("positions must be within range and distinct");
            }
            nextRoad_[i][pos] = 0;
        }
    }

    for (int i = 0; i < params_.lanes; ++i) {
        road_[i].swap(nextRoad_[i]);
    }

    left2right = true;
    assert(checkCount());
}

void TrafficSim::step() { //
    if (params_.lanes > 1) {
        changeLanes();
    }
    forward();
}

void TrafficSim::changeLanes() {
    laneChanges_ = 0;
    // Nearest leader speed within the lookahead; absence is infinity.
    const auto leaderSpeed = [this](const int lane, const int pos) {
        for (int k = 1; k <= params_.laneChangeLookahead; ++k) {
            const int cell = road_[lane][wrapForward(pos, k, params_.length)];
            if (cell >= 0) {
                return cell;
            }
        }
        return std::numeric_limits<int>::max();
    };

    const auto security = [this](const int lane, const int pos, const int targetLane) {
        // [-vMax, v]
        const int v = road_[lane][pos];
        // find car infront
        for (int k = 0; k <= v; ++k) {
            if (road_[targetLane][wrapForward(pos, k, params_.length)] >= 0) {
                return false;
            }
        }
        // find car behind
        for (int k = 1; k <= params_.vMax; ++k) {
            if (road_[targetLane][wrapBackward(pos, k, params_.length)] >= 0) {
                return false;
            }
        }
        return true;
    };

    const auto incentiveL2R = [this, leaderSpeed](const int lane, const int pos, const int targetLane) {
        const int currentLeader = leaderSpeed(lane, pos);
        const int targetLeader = leaderSpeed(targetLane, pos);
        const int v = road_[lane][pos];
        return (currentLeader <= v) || (targetLeader <= v);
    };

    const auto incentiveR2L = [this, leaderSpeed](const int lane, const int pos, const int targetLane) {
        const int currentLeader = leaderSpeed(lane, pos);
        const int targetLeader = leaderSpeed(targetLane, pos);
        const int v = road_[lane][pos];
        return (currentLeader > v + params_.slack) && (targetLeader > v + params_.slack);
    };

    for (std::vector<int> &lane : nextRoad_) {
        std::ranges::fill(lane, -1);
    }

    int startLane;
    int endLane;
    if (left2right) {
        startLane = 0;
        endLane = params_.lanes - 1;
        nextRoad_[params_.lanes - 1] = road_[params_.lanes - 1];
    } else {
        startLane = 1;
        endLane = params_.lanes;
        nextRoad_[0] = road_[0];
    }

    for (int lane = startLane; lane < endLane; ++lane) {
        for (int pos = 0; pos < params_.length; ++pos) {
            const int v = road_[lane][pos];
            if (v < 0) {
                continue;
            }

            int destLane = lane;
            int targetLane = left2right ? lane + 1 : lane - 1;
            if (security(lane, pos, targetLane)) {
                const bool change =
                    left2right ? incentiveL2R(lane, pos, targetLane) : incentiveR2L(lane, pos, targetLane);
                destLane = change ? targetLane : lane;
            }

            assert(nextRoad_[destLane][pos] < 0);
            laneChanges_ += (destLane != lane);
            nextRoad_[destLane][pos] = v;
        }
    }

    for (int lane = 0; lane < params_.lanes; ++lane) {
        road_[lane].swap(nextRoad_[lane]);
    }

    left2right = !left2right;

    assert(checkCount());
}

void TrafficSim::forward() {
    for (int lane = 0; lane < params_.lanes; ++lane) {
        std::ranges::fill(nextRoad_[lane], -1);

        for (int n = 0; n < params_.length; ++n) {
            if (road_[lane][n] < 0) {
                continue;
            }

            // accelerate.
            int v = std::min(road_[lane][n] + 1, params_.vMax);

            // brake limit
            for (int k = 1; k <= v; ++k) {
                if (road_[lane][wrapForward(n, k, params_.length)] >= 0) {
                    v = k - 1;
                    break;
                }
            }

            // random
            if (brakeDist_(rng_)) {
                v = std::max(v - 1, 0);
            }

            // update
            n += v; // sorted so can skip to next car
            nextRoad_[lane][wrapForward(n, 0, params_.length)] = v;
        }

        road_[lane].swap(nextRoad_[lane]);
    }
    assert(checkCount());
}

int TrafficSim::countLane(const std::vector<int> &lane) const {
    const auto isCar = [](const int &cell) { return cell >= 0; };
    return std::ranges::count_if(lane, isCar);
}

bool TrafficSim::checkCount() const { //
    int total = 0;
    for (int i = 0; i < params_.lanes; ++i) {
        total += countLane(road_[i]);
    }
    return total == numCars_;
}

int TrafficSim::sumVelocity(const int lane) const {
    if (lane < 0 || lane >= params_.lanes) {
        throw std::out_of_range("lane index out of range");
    }

    auto filterCar = [](const int &cell) { return std::max(cell, 0); };

    return std::transform_reduce(road_[lane].begin(), road_[lane].end(), 0, std::plus{}, filterCar);
}

double TrafficSim::avgVelocity() const {
    if (numCars_ == 0) {
        return 0.0;
    }

    int total = 0;
    for (int lane = 0; lane < params_.lanes; ++lane) {
        total += sumVelocity(lane);
    }
    return static_cast<double>(total) / numCars_;
}

double TrafficSim::avgVelocity(const int lane) const {
    if (lane < 0 || lane >= params_.lanes) {
        throw std::out_of_range("lane index out of range");
    }

    const int cars = countLane(road_[lane]);
    if (cars == 0) {
        return 0.0;
    }

    return static_cast<double>(sumVelocity(lane)) / cars;
}

double TrafficSim::laneUsage(const int lane) const {
    if (lane < 0 || lane >= params_.lanes) {
        throw std::out_of_range("lane index out of range");
    }
    return numCars_ == 0 ? 0.0 : static_cast<double>(countLane(road_[lane])) / numCars_;
}

double TrafficSim::flowrate() const {
    std::int64_t total = 0;
    for (int lane = 0; lane < params_.lanes; ++lane) {
        total += sumVelocity(lane);
    }
    return static_cast<double>(total) / params_.length;
}

double TrafficSim::flowrate(const int lane) const {
    if (lane < 0 || lane >= params_.lanes) {
        throw std::out_of_range("lane index out of range");
    }
    return static_cast<double>(sumVelocity(lane)) / params_.length;
}

double TrafficSim::laneChangeFrequency() const {
    if (numCars_ == 0) {
        return 0.0;
    }
    return static_cast<double>(laneChanges_) / numCars_;
}

void TrafficSim::print(std::ostream &os) const {
    for (const std::vector<int> &lane : road_) {
        for (const int &cell : lane) {
            if (cell < 0) {
                os << " ";
            } else {
                os << cell;
            }
        }
        os << "\n";
    }
    os << "\n";
}

} // namespace traffic
