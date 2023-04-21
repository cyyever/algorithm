/*!
 * \file list_algorithm.hpp
 *
 * \brief list algorithms
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

#include <cstddef>
#include <list>
#include <optional>
#include <ranges>
#include <span>
#include <unordered_map>
#include <vector>

namespace cyy::algorithm {

  template <typename T>
  std::pair<std::list<T>, std::list<T>> half_cut_list(std::list<T> L) {
    auto mid = L.size() / 2;
    auto mid_it = L.begin();
    std::advance(mid_it, mid);
    std::list<T> first_half;
    first_half.splice(first_half.end(), L, L.begin(), mid_it);
    std::list<T> second_half;
    second_half.splice(second_half.end(), std::move(L));
    return {std::move(first_half), std::move(second_half)};
  }

  // find the number of inversions
  template <std::ranges::input_range U> size_t get_inversion_number(U A) {
    using list_type = std::list<std::ranges::range_value_t<U>>;
    auto sort_and_count = [](auto &&self,
                             list_type l) -> std::pair<list_type, size_t> {
      auto span_size = l.size();
      if (span_size <= 1) {
        return {std::move(l), 0};
      }
      auto [first_half, second_half] = half_cut_list(std::move(l));

      auto [sorted_first_half, count1] = self(self, std::move(first_half));
      auto [sorted_second_half, count2] = self(self, std::move(second_half));
      size_t count3 = 0;
      list_type sorted_list;
      while (!sorted_first_half.empty() && !sorted_second_half.empty()) {
        if (sorted_first_half.front() <= sorted_second_half.front()) {
          sorted_list.splice(sorted_list.end(), sorted_first_half,
                             sorted_first_half.begin());
        } else {
          count3 += sorted_first_half.size();
          sorted_list.splice(sorted_list.end(), sorted_second_half,
                             sorted_second_half.begin());
        }
      }
      if (!sorted_first_half.empty()) {
        sorted_list.splice(sorted_list.end(), std::move(sorted_first_half));
      }
      if (!sorted_second_half.empty()) {
        sorted_list.splice(sorted_list.end(), std::move(sorted_second_half));
      }
      return {sorted_list, count3 + count1 + count2};
    };
    list_type l(std::ranges::begin(A), std::ranges::end(A));
    if (l.size() <= 1) {
      return 0;
    }
    return sort_and_count(sort_and_count, l).second;
  }

} // namespace cyy::algorithm
