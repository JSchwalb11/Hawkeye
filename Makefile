BUILD_TYPE ?= Debug
BUILD_DIR  := build
CMAKE_EXTRA ?=

# Cross-platform parallel job count
ifeq ($(OS),Windows_NT)
    JOBS := $(NUMBER_OF_PROCESSORS)
    RM   := rmdir /s /q
    EXE  := $(BUILD_DIR)\hawkeye.exe
else
    JOBS := $(shell sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
    RM   := rm -rf
    EXE  := $(BUILD_DIR)/hawkeye
endif

.PHONY: build configure test clean release run test-core sanitize fixtures renders

build: configure
	cmake --build $(BUILD_DIR) --config $(BUILD_TYPE) -j$(JOBS)

configure:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DBUILD_TESTING=ON $(CMAKE_EXTRA)

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure -C $(BUILD_TYPE)

release:
	$(MAKE) BUILD_TYPE=Release

# Core tests only (no raylib) — fast CI path
test-core:
	$(MAKE) CMAKE_EXTRA="-DBUILD_TESTING_ONLY=ON"

# Fleet-map fixtures: record a tlog per fixture, replay it, and score the map
# against the ground truth the injector published. No raylib, no GPU, no live
# process -- which is what keeps them usable in CI.
fixtures:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
	      -DBUILD_TESTING_ONLY=ON $(CMAKE_EXTRA)
	cmake --build $(BUILD_DIR) --target ray_injector map_checker \
	      test_no_fixture_symbols -j$(JOBS)
	ctest --test-dir $(BUILD_DIR) --output-on-failure -R "fixture_|no_fixture_symbols"

# Orthographic renders of the fixtures, for the docs and for pull requests.
renders: fixtures
	@mkdir -p docs/assets/fleet-map
	@for f in ground two-origins disagreement corridor vanishing clocks firehose; do \
		$(BUILD_DIR)/test/ray_injector --fixture $$f \
			--tlog $(BUILD_DIR)/test/fixture-runs/$$f.tlog \
			--truth $(BUILD_DIR)/test/fixture-runs/$$f.truth >/dev/null; \
		$(BUILD_DIR)/test/map_checker \
			--truth $(BUILD_DIR)/test/fixture-runs/$$f.truth \
			--tlog $(BUILD_DIR)/test/fixture-runs/$$f.tlog \
			--render docs/assets/fleet-map/$$f.png >/dev/null; \
	done
	@echo "renders written to docs/assets/fleet-map/"

# Address + undefined behavior sanitizers
sanitize:
	$(MAKE) CMAKE_EXTRA="-DBUILD_TESTING_ONLY=ON -DCMAKE_C_FLAGS=\"-fsanitize=address,undefined -fno-omit-frame-pointer\""

clean:
	$(RM) $(BUILD_DIR)

run: build
	$(EXE)
