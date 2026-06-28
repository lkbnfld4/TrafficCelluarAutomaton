#pragma once

#include <iosfwd>
#include <random>
#include <span>
#include <vector>

namespace traffic {

// parameters for TrafficSim.
struct SimParams {
    int length = 10'000;     // length of the road.
    double density = 0.3;    // car density for cells occupied.
    int vMax = 5;            // maximum velocity of car.
    double brakeProb = 0.15; // random braking probability
    int seed = 0;            // rng seed; 0 is random seed.

    // returns these parameters or throws std::invalid_argument
    SimParams validate() const;
};

// Nagel-Schreckenberg cellular automaton on a circular single road.
class TrafficSim {
public:
    explicit TrafficSim(const SimParams &params);

    // distributes the cars randomly with velocity 0.
    void setup();

    // places the cars on the given cells with velocity 0.
    void setup(std::span<const int> positions);

    // update sim a timestep.
    void step();

    // cell holds -1 when empty, or a car with velocity in [0, vMax].
    std::vector<int> road() const { return road_; }

    // number of cars
    int numCars() const { return numCars_; }

    // check number of cars still matches the initial count.
    bool checkCount() const;

    // writes the road as one character per cell
    void print(std::ostream &os) const;

private:
    void place(std::span<const int> positions);

    const SimParams params_;
    std::vector<int> road_;
    std::vector<int> nextRoad_;
    int numCars_;
    std::bernoulli_distribution brakeDist_;
    std::mt19937 rng_;
};

} // namespace traffic
