#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <ranges>
#include <vector>

class TrafficSim {
public:
  TrafficSim(int N, double rho, int V, double p, int seed = 0)
      : length(N), road_(N, -1), nextRoad_(N, -1), density(rho),
        numCars(std::round(N * rho)), vMax(V), breakProb(p), brakeDist(p),
        rng(seed == 0 ? std::random_device{}() : seed) {}

  void setup() {
    std::vector<int> start(numCars);
    auto range = std::views::iota(0, length);
    std::ranges::sample(range, start.begin(), numCars, rng);

    std::ranges::fill(road_, -1);
    for (int i : start) {
      road_[i] = 0;
    }
  }

  void step() {
    std::ranges::fill(nextRoad_, -1);

    for (int n = 0; n < length; ++n) {
      if (road_[n] < 0) {
        continue;
      };

      // accelerate
      auto v = std::min(road_[n] + 1, vMax);

      // limit
      for (int k = 1; k <= v; ++k) {
        if (road_[(n + k) % length] >= 0) {
          v = k - 1;
          break;
        }
      }

      // random
      if (brakeDist(rng)) {
        v = std::max(v - 1, 0);
      }

      // update
      n += v; // sorted so can skip to next car
      int new_n = n % length;
      nextRoad_[new_n] = v;
    }
    std::swap(road_, nextRoad_);
    assert(checkCount());
  }

  bool checkCount() const {
    auto isCar = [](const int &num) { return num >= 0; };

    return std::ranges::count_if(road_, isCar) == numCars;
  }

  void printRoad() const {
    for (auto &c : road_) {
      if (c < 0) {
        std::cout << " ";
      } else {
        std::cout << c;
      }
    }
    std::cout << std::endl;
  }

private:
  int length;
  std::vector<int> road_, nextRoad_;
  double density;
  int numCars;
  int vMax;
  double breakProb;
  std::bernoulli_distribution brakeDist;
  std::mt19937 rng;
};

int main(int argc, char *argv[]) {
  int N = 10000;
  int T = 5000;
  double rho = 0.3;
  int vMax = 5;
  double p = 0.15;

  TrafficSim sim(N, rho, vMax, p);
  sim.setup();

  // sim.printRoad();
  auto t1 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < T; ++i) {
    sim.step();
    // sim.printRoad();
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms_double = t2 - t1;
  std::cout << ms_double.count() << "ms" << std::endl;

  return 0;
}