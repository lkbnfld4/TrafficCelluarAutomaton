CXXFLAGS := -std=c++20 -O3 -march=native -Wall -Wextra

all: traffic

traffic: traffic.cpp
	g++ $(CXXFLAGS) traffic.cpp -o traffic

run: traffic
	./traffic

clean:
	rm -f traffic

.PHONY: all run clean
