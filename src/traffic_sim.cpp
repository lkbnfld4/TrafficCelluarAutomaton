#include "traffic/traffic_sim.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <ostream>
#include <ranges>
#include <stdexcept>

namespace traffic {

SimParams SimParams::validate() const {
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
    return *this;
}

// clang-format off
TrafficSim::TrafficSim(const SimParams &params)
    : params_(params.validate()),
      road_(params.length, -1),
      nextRoad_(params.length, -1),
      numCars_(std::lround(params.length * params.density)),
      brakeDist_(params.brakeProb),
      rng_(params.seed == 0 ? std::random_device{}() : params.seed) {}
// clang-format on

void TrafficSim::setup() {
    std::vector<int> start(numCars_);
    std::ranges::sample(std::views::iota(0, params_.length), start.begin(), numCars_, rng_);

    place(start);
}

void TrafficSim::setup(const std::span<const int> positions) { //
    place(positions);
}

void TrafficSim::place(const std::span<const int> positions) {
    if (positions.size() != static_cast<std::size_t>(numCars_)) {
        throw std::invalid_argument("positions requires numCars");
    }

    std::ranges::fill(road_, -1);
    for (const int &pos : positions) {
        if (pos < 0 || pos >= params_.length || road_[pos] >= 0) {
            throw std::invalid_argument("positions must be within range and distinct");
        }
        road_[pos] = 0;
    }
}

void TrafficSim::step() {
    std::ranges::fill(nextRoad_, -1);

    for (int n = 0; n < params_.length; ++n) {
        if (road_[n] < 0) {
            continue;
        }

        // accelerate.
        int v = std::min(road_[n] + 1, params_.vMax);

        // break limit
        for (int k = 1; k <= v; ++k) {
            if (road_[(n + k) % params_.length] >= 0) {
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
        nextRoad_[n % params_.length] = v;
    }

    road_.swap(nextRoad_);
    assert(checkCount());
}

bool TrafficSim::checkCount() const {
    const auto isCar = [](const int &cell) { return cell >= 0; };
    return std::ranges::count_if(road_, isCar) == numCars_;
}

void TrafficSim::print(std::ostream &os) const {
    for (const int &cell : road_) {
        if (cell < 0) {
            os << " ";
        } else {
            os << cell;
        }
    }
    os << "\n";
}

} // namespace traffic
