# ==============================================================================
# Standalone Linux Host Makefile for Native Test Execution
# Project: Industrial Predictive Maintenance for DC Motor
# ==============================================================================
CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -DNATIVE_BUILD -DUNIT_TEST -Iinclude -Itest/mocks -Itest
SANITIZE_FLAGS = -fsanitize=address,undefined -g

# Discover source and test files dynamically
SRC_FILES = $(wildcard src/*.cpp)
TEST_FILES = $(wildcard test/*.cpp)

TARGET = bin/run_tests
TARGET_SAN = bin/run_tests_sanitized

all: $(TARGET)

$(TARGET): $(SRC_FILES) $(TEST_FILES)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SRC_FILES) $(TEST_FILES) -o $(TARGET)

sanitize: $(SRC_FILES) $(TEST_FILES)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(SANITIZE_FLAGS) $(SRC_FILES) $(TEST_FILES) -o $(TARGET_SAN)
	./$(TARGET_SAN)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -rf bin/

.PHONY: all sanitize test clean
