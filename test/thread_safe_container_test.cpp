/*!
 * \file container_test.cpp
 *
 * \brief 测试container相关函数
 * \author cyy
 */
#include <chrono>
#include <functional>
#include <thread>

#include <doctest/doctest.h>

#include "thread_safe_container.hpp"

TEST_CASE("thread_safe_linear_container") {
  cyy::algorithm::thread_safe_linear_container<std::vector<int>> container;

  CHECK(container.const_ref()->empty());

  SUBCASE("push_back") {
    container.push_back(1);
    CHECK(!container.const_ref()->empty());
    CHECK_GT(container.const_ref()->size(), 0);
    container.clear();
    CHECK(container.const_ref()->empty());
  }

  SUBCASE("concurrently push_back") {
    container.clear();
    {
      std::vector<std::jthread> thds;
      thds.reserve(10);
      for (int i = 0; i < 10; i++) {
        thds.emplace_back([i, &container]() { container.push_back(i); });
      }
    }
    CHECK_EQ(container.const_ref()->size(), 10);
    container.clear();
    CHECK(container.const_ref()->empty());
  }

  SUBCASE("back") {
    using namespace std::chrono_literals;
    container.clear();
    container.push_back(1);
    auto val = container.back(1s);
    CHECK_EQ(val, 1);
    container.pop_front();
    container.push_back(2);
    val = container.pop_front(1us);
    CHECK_EQ(val, 2);
    val = container.back(1us);
    CHECK(!val.has_value());
    container.clear();
    CHECK(container.const_ref()->empty());
  }
  SUBCASE("back") {
    using namespace std::chrono_literals;
    container.clear();
    CHECK(container.const_ref()->empty());

    std::stop_source source;
    std::jthread const pop_thd(
        [&container, &source] { container.back(1ms, source.get_token()); });
    source.request_stop();
    std::jthread const back_thd([&container] {
      auto val = container.back(1000ms);
      CHECK_EQ(val, 5);
    });
    container.push_back(5);
  }
  SUBCASE("batch_pop_front") {
    container.emplace_back(1);
    container.emplace_back(2);
    auto res = container.batch_pop_front(2, std::chrono::milliseconds(1));
    REQUIRE_EQ(res, std::vector<int>{1, 2});
    res = container.batch_pop_front(2, std::chrono::milliseconds(1));
    REQUIRE(res.empty());
  }
  SUBCASE("concurrently pop_front") {
    std::vector<std::jthread> thds;

    thds.reserve(10);
    for (int i = 0; i < 10; i++) {
      thds.emplace_back([&container]() {
        CHECK(!container.pop_front(std::chrono::microseconds(1)).has_value());
      });
    }
  }
  SUBCASE("concurrently push_back and pop_front") {
    container.clear();
    {
      std::vector<std::jthread> thds;
      for (int i = 0; i < 500; i++) {
        thds.emplace_back([i, &container]() { container.push_back(i); });
        thds.emplace_back([&container]() {
          CHECK(container.pop_front(std::chrono::minutes(1)).has_value());
        });
      }
    }
    CHECK(container.const_ref()->empty());
  }
}
