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

.PHONY: build configure test clean release run test-core sanitize fixtures renders videos

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
	@for f in statue-solo statue-fleet; do \
		$(BUILD_DIR)/test/ray_injector --fixture $$f \
			--splat assets/statue_of_liberty.splat \
			--mesh assets/statue_of_liberty.tri \
			--tlog $(BUILD_DIR)/test/fixture-runs/$$f.tlog \
			--truth $(BUILD_DIR)/test/fixture-runs/$$f.truth >/dev/null; \
		$(BUILD_DIR)/test/map_checker \
			--truth $(BUILD_DIR)/test/fixture-runs/$$f.truth \
			--tlog $(BUILD_DIR)/test/fixture-runs/$$f.tlog \
			--render docs/assets/fleet-map/$$f.png \
			--gif docs/assets/fleet-map/$$f-mapping.gif \
			--gif-interval 1.0 --gif-size 440 620 \
			--gif-view side --gif-delay 9 >/dev/null; \
	done
	@echo "renders written to docs/assets/fleet-map/"

# Full-resolution PNG frames of the map filling in, and an h264 encode of them.
# Kept out of `renders` because it needs ffmpeg, which the fixtures do not.
FFMPEG ?= ffmpeg
videos: fixtures
	@mkdir -p docs/assets/fleet-map
	@for f in statue-solo statue-fleet; do \
		rm -rf $(BUILD_DIR)/frames-$$f && mkdir -p $(BUILD_DIR)/frames-$$f; \
		$(BUILD_DIR)/test/map_checker \
			--truth $(BUILD_DIR)/test/fixture-runs/$$f.truth \
			--tlog $(BUILD_DIR)/test/fixture-runs/$$f.tlog \
			--frames $(BUILD_DIR)/frames-$$f \
			--gif-interval 0.2 --gif-size 720 1000 --gif-view side >/dev/null; \
		$(FFMPEG) -y -framerate 24 -i $(BUILD_DIR)/frames-$$f/frame-%05d.png \
			-c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p \
			-movflags +faststart docs/assets/fleet-map/$$f-mapping.mp4; \
	done
	@echo "videos written to docs/assets/fleet-map/"

# Address + undefined behavior sanitizers
sanitize:
	$(MAKE) CMAKE_EXTRA="-DBUILD_TESTING_ONLY=ON -DCMAKE_C_FLAGS=\"-fsanitize=address,undefined -fno-omit-frame-pointer\""

clean:
	$(RM) $(BUILD_DIR)

run: build
	$(EXE)
