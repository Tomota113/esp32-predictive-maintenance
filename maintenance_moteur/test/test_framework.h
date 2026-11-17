#pragma once
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <algorithm>

// =============================================================================
// Lightweight Zero-Dependency C++17 Test Framework & Test Runner Harness
// =============================================================================

namespace TestHarness {

// ANSI Color Codes
constexpr const char* COLOR_RESET   = "\033[0m";
constexpr const char* COLOR_RED     = "\033[0;31m";
constexpr const char* COLOR_GREEN   = "\033[0;32m";
constexpr const char* COLOR_YELLOW  = "\033[0;33m";
constexpr const char* COLOR_BLUE    = "\033[0;34m";
constexpr const char* COLOR_CYAN    = "\033[0;36m";
constexpr const char* COLOR_BOLD    = "\033[1m";

class AssertionException : public std::exception {
private:
    std::string message;
    std::string file;
    int line;
    std::string full_info;

public:
    AssertionException(std::string msg, std::string f, int l)
        : message(std::move(msg)), file(std::move(f)), line(l) {
        std::ostringstream ss;
        ss << file << ":" << line << ": Failure\n" << message;
        full_info = ss.str();
    }

    const char* what() const noexcept override {
        return full_info.c_str();
    }

    const std::string& get_message() const { return message; }
    const std::string& get_file() const { return file; }
    int get_line() const { return line; }
};

struct TestCase {
    std::string suite_name;
    std::string test_name;
    std::function<void()> test_fn;
    std::string file;
    int line;
};

class Registry {
private:
    std::vector<TestCase> tests;

    Registry() = default;

public:
    static Registry& instance() {
        static Registry reg;
        return reg;
    }

    void add_test(const std::string& suite, const std::string& name,
                  const std::string& file, int line, std::function<void()> fn) {
        tests.push_back({suite, name, std::move(fn), file, line});
    }

    const std::vector<TestCase>& get_tests() const {
        return tests;
    }
};

struct TestRegistrar {
    TestRegistrar(const std::string& suite, const std::string& name,
                  const std::string& file, int line, std::function<void()> fn) {
        Registry::instance().add_test(suite, name, file, line, std::move(fn));
    }
};

class Runner {
public:
    static int run(int argc = 0, char** argv = nullptr) {
        std::string filter = "";
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.rfind("--filter=", 0) == 0) {
                filter = arg.substr(9);
            }
        }

        const auto& all_tests = Registry::instance().get_tests();
        std::vector<TestCase> selected_tests;

        for (const auto& t : all_tests) {
            std::string full_name = t.suite_name + "." + t.test_name;
            if (filter.empty() || full_name.find(filter) != std::string::npos) {
                selected_tests.push_back(t);
            }
        }

        std::cout << COLOR_CYAN << "[==========]" << COLOR_RESET << " Running "
                  << selected_tests.size() << " tests from test suites.\n";

        size_t passed_count = 0;
        size_t failed_count = 0;
        std::vector<std::pair<std::string, std::string>> failures;

        auto total_start = std::chrono::high_resolution_clock::now();

        for (const auto& t : selected_tests) {
            std::string full_name = t.suite_name + "." + t.test_name;
            std::cout << COLOR_CYAN << "[ RUN      ]" << COLOR_RESET << " " << full_name << "\n";

            auto test_start = std::chrono::high_resolution_clock::now();
            bool passed = false;
            std::string failure_msg = "";

            try {
                t.test_fn();
                passed = true;
            } catch (const AssertionException& e) {
                failure_msg = e.what();
            } catch (const std::exception& e) {
                failure_msg = std::string("Unexpected standard exception: ") + e.what();
            } catch (...) {
                failure_msg = "Unknown exception caught during test execution.";
            }

            auto test_end = std::chrono::high_resolution_clock::now();
            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(test_end - test_start).count();

            if (passed) {
                passed_count++;
                std::cout << COLOR_GREEN << "[       OK ]" << COLOR_RESET << " " << full_name
                          << " (" << duration_ms << " ms)\n";
            } else {
                failed_count++;
                failures.emplace_back(full_name, failure_msg);
                std::cout << COLOR_RED << "[  FAILED  ]" << COLOR_RESET << " " << full_name
                          << " (" << duration_ms << " ms)\n";
                std::cout << COLOR_RED << "  " << failure_msg << COLOR_RESET << "\n";
            }
        }

        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();

        std::cout << COLOR_CYAN << "[==========]" << COLOR_RESET << " "
                  << selected_tests.size() << " tests ran. (" << total_ms << " ms total)\n";
        std::cout << COLOR_GREEN << "[  PASSED  ]" << COLOR_RESET << " "
                  << passed_count << " tests.\n";

        if (failed_count > 0) {
            std::cout << COLOR_RED << "[  FAILED  ]" << COLOR_RESET << " "
                      << failed_count << " tests, listed below:\n";
            for (const auto& f : failures) {
                std::cout << COLOR_RED << "[  FAILED  ]" << COLOR_RESET << " " << f.first << "\n";
            }
            return 1;
        }

        return 0;
    }
};

} // namespace TestHarness

// =============================================================================
// Test Definition Macro
// =============================================================================

#define TEST(suite_name, test_name)                                                                   \
    void suite_name##_##test_name##_Test();                                                           \
    static ::TestHarness::TestRegistrar suite_name##_##test_name##_Registrar(                         \
        #suite_name, #test_name, __FILE__, __LINE__, suite_name##_##test_name##_Test);                \
    void suite_name##_##test_name##_Test()

#define RUN_ALL_TESTS(argc, argv) ::TestHarness::Runner::run(argc, argv)

// =============================================================================
// Assertion Macros
// =============================================================================

#define ASSERT_TRUE(condition)                                                                         \
    do {                                                                                               \
        if (!(condition)) {                                                                            \
            std::ostringstream _ss;                                                                    \
            _ss << "Value of: " #condition "\n  Actual: false\nExpected: true";                        \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_FALSE(condition)                                                                        \
    do {                                                                                               \
        if (condition) {                                                                               \
            std::ostringstream _ss;                                                                    \
            _ss << "Value of: " #condition "\n  Actual: true\nExpected: false";                       \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_EQ(val1, val2)                                                                          \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        if (!(_v1 == _v2)) {                                                                           \
            std::ostringstream _ss;                                                                    \
            _ss << "Expected equality of these values:\n"                                              \
                << "  " #val1 "\n    Which is: " << _v1 << "\n"                                       \
                << "  " #val2 "\n    Which is: " << _v2;                                               \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_NE(val1, val2)                                                                          \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        if (_v1 == _v2) {                                                                              \
            std::ostringstream _ss;                                                                    \
            _ss << "Expected inequality of these values:\n"                                            \
                << "  " #val1 "\n    Which is: " << _v1 << "\n"                                       \
                << "  " #val2 "\n    Which is: " << _v2;                                               \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_NEAR(val1, val2, abs_error)                                                             \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        auto _diff = std::abs(_v1 - _v2);                                                              \
        auto _tol = (abs_error);                                                                       \
        if (_diff > _tol) {                                                                            \
            std::ostringstream _ss;                                                                    \
            _ss << "The difference between (" #val1 ") and (" #val2 ") is " << _diff                  \
                << ", which exceeds " #abs_error " (" << _tol << ").\n"                                \
                << "  " #val1 ": " << _v1 << "\n  " #val2 ": " << _v2;                                 \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_LT(val1, val2)                                                                          \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        if (!(_v1 < _v2)) {                                                                            \
            std::ostringstream _ss;                                                                    \
            _ss << "Expected: (" #val1 ") < (" #val2 ")\n"                                             \
                << "  Actual: " << _v1 << " vs " << _v2;                                               \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_LE(val1, val2)                                                                          \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        if (!(_v1 <= _v2)) {                                                                           \
            std::ostringstream _ss;                                                                    \
            _ss << "Expected: (" #val1 ") <= (" #val2 ")\n"                                            \
                << "  Actual: " << _v1 << " vs " << _v2;                                               \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_GT(val1, val2)                                                                          \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        if (!(_v1 > _v2)) {                                                                            \
            std::ostringstream _ss;                                                                    \
            _ss << "Expected: (" #val1 ") > (" #val2 ")\n"                                             \
                << "  Actual: " << _v1 << " vs " << _v2;                                               \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_GE(val1, val2)                                                                          \
    do {                                                                                               \
        auto _v1 = (val1);                                                                             \
        auto _v2 = (val2);                                                                             \
        if (!(_v1 >= _v2)) {                                                                           \
            std::ostringstream _ss;                                                                    \
            _ss << "Expected: (" #val1 ") >= (" #val2 ")\n"                                            \
                << "  Actual: " << _v1 << " vs " << _v2;                                               \
            throw ::TestHarness::AssertionException(_ss.str(), __FILE__, __LINE__);                    \
        }                                                                                              \
    } while (0)

#define ASSERT_FLOAT_EQ(val1, val2) ASSERT_NEAR(val1, val2, 1e-5f)
#define ASSERT_DOUBLE_EQ(val1, val2) ASSERT_NEAR(val1, val2, 1e-9)

// Non-fatal EXPECT macros aliased to ASSERT in this lightweight harness
#define EXPECT_TRUE(cond) ASSERT_TRUE(cond)
#define EXPECT_FALSE(cond) ASSERT_FALSE(cond)
#define EXPECT_EQ(v1, v2) ASSERT_EQ(v1, v2)
#define EXPECT_NE(v1, v2) ASSERT_NE(v1, v2)
#define EXPECT_NEAR(v1, v2, tol) ASSERT_NEAR(v1, v2, tol)
#define EXPECT_LT(v1, v2) ASSERT_LT(v1, v2)
#define EXPECT_LE(v1, v2) ASSERT_LE(v1, v2)
#define EXPECT_GT(v1, v2) ASSERT_GT(v1, v2)
#define EXPECT_GE(v1, v2) ASSERT_GE(v1, v2)
