#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <ranges>
#include <vector>

void printVector(std::vector<int> road) {

  for (auto c : road) {
    if (c < 0) {
      std::cout << " ";
    } else {
      std::cout << c;
    }
  }
  std::cout << std::endl;
}

bool isCar(const int &num) { return num >= 0; }

std::vector<int> step(std::vector<int> &road, const int &vMax, const double &p,
                      std::default_random_engine &generator) {
  std::vector<int> newRoad(road.size(), -1);
  std::bernoulli_distribution distribution(p);

  for (int n = 0; n < road.size(); ++n) {
    if (road[n] < 0) {
      continue;
    };

    // accelerate
    auto v = std::min(road[n] + 1, vMax);

    // limit
    for (int k = 1; k <= v; ++k) {
      if (road[(n + k) % road.size()] >= 0) {
        v = k - 1;
        break;
      }
    }

    // random
    if (distribution(generator)) {
      v = std::max(v - 1, 0);
    }

    // update
    n += v; // sorted so can skip to next car
    int new_n = n % road.size();
    newRoad[new_n] = v;
  }

  return newRoad;
}

int main(int argc, char *argv[]) {
  int N = 10000;
  int T = 5000;
  double rho = 0.3;
  int vMax = 5;
  double p = 0.15;

  int M = std::round(N * rho);
  std::vector<int> start(M);

  std::default_random_engine generator;
  auto range = std::views::iota(0, N);
  std::ranges::sample(range, start.begin(), M, generator);

  std::vector<int> road(N, -1);
  for (int i : start) {
    road[i] = 0;
  }

  // printVector(road);
  auto t1 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < T; ++i) {
    road = step(road, vMax, p, generator);
    // printVector(road);
    assert(std::ranges::count_if(road.begin(), road.end(), isCar) == M);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms_double = t2 - t1;
  std::cout << ms_double.count() << "ms" << std::endl;

  return 0;
}