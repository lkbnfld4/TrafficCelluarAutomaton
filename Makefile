BUILD_DIR  ?= build
BUILD_TYPE ?= Release
ARGS       ?= 80 0.3 4 0.2 100

all:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	cmake --build $(BUILD_DIR) --parallel

run: all
	./$(BUILD_DIR)/traffic $(ARGS)

test:
	cmake -S . -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DBUILD_TESTING=ON
	cmake --build $(BUILD_DIR) --parallel
	ctest --test-dir $(BUILD_DIR) --output-on-failure -LE benchmark

test-debug:
	cmake -S . -B $(BUILD_DIR)-debug -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
	cmake --build $(BUILD_DIR)-debug --parallel
	ctest --test-dir $(BUILD_DIR)-debug --output-on-failure -LE benchmark

check: test test-debug

bench: test
	ctest --test-dir $(BUILD_DIR) -L benchmark -V

clean:
	rm -rf $(BUILD_DIR) $(BUILD_DIR)-debug

.PHONY: all run test test-debug check bench clean
