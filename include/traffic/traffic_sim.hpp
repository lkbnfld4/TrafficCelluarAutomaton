#pragma once

#include <iosfwd>
#include <random>
#include <vector>

namespace traffic {

// parameters for TrafficSim.
struct SimParams {
    int lanes = 1;
    int length = 10'000;         // length of the road.
    double density = 0.2;        // car density for cells occupied.
    int vMax = 5;                // maximum velocity of car.
    double brakeProb = 0.2;      // random braking probability
    int laneChangeLookahead = 7; // cells ahead in lane-change rule
    int slack = 3;               // slack delay for lane-change rule
    int seed = 0;                // rng seed; 0 is random seed.

    // returns these parameters or throws std::invalid_argument
    SimParams validate() const;
};

// Nagel-Schreckenberg cellular automaton on a circular multi-lane road.
class TrafficSim {
public:
    const SimParams params_;

    explicit TrafficSim(const SimParams &params);

    // distributes the cars randomly with velocity 0.
    void setup();

    // places the cars on the given cells with velocity 0.
    void setup(const std::vector<std::vector<int>> &positions);

    // update sim a timestep.
    void step();

    // lane 0 is the leftmost/default lane
    // cell holds -1 when empty, or a car with velocity in [0, vMax].
    const std::vector<std::vector<int>> &road() const { return road_; }

    // number of cars
    int numCars() const { return numCars_; }

    // check number of cars still matches the initial count.
    bool checkCount() const;

    // average over all lanes, zero when empty.
    double avgVelocity() const;
    // average over the cars in lane; zero when empty.
    double avgVelocity(int lane) const;

    // Fraction of all cars in the given lane, or zero when empty.
    double laneUsage(int lane) const;

    // Sum of per-lane flows across the road.
    double flowrate() const;
    // Sum of velocities in the lane divided by the lane length.
    double flowrate(int lane) const;

    // Lane changes in the most recently completed step, per car.
    double laneChangeFrequency() const;

    // writes the road as one character per cell
    void print(std::ostream &os) const;

private:
    void place(const std::vector<std::vector<int>> &positions);
    void forward();
    void changeLanes();

    int countLane(const std::vector<int> &lane) const;
    int sumVelocity(int lane) const;

    std::vector<std::vector<int>> road_;
    std::vector<std::vector<int>> nextRoad_;
    int numCars_;
    int laneChanges_ = 0;
    bool left2right = true;
    std::bernoulli_distribution brakeDist_;
    std::mt19937 rng_;
};

} // namespace traffic
