/*!
 * \file hash.hpp
 *
 */

#pragma once

#include <boost/container_hash/hash.hpp>
#include <boost/pfr.hpp>
import std;

namespace std {
  template <typename T>
  concept Hashable = requires(T a) {
    { std::hash<T>{}(a) } -> std::convertible_to<std::size_t>;
  };

  template <Hashable T1, Hashable T2> struct hash<std::pair<T1, T2>> {
    std::size_t operator()(const std::pair<T1, T2> &x) const noexcept {
      return boost::hash<decltype(x)>()(x);
    }
  };

  template <std::ranges::input_range T>
    requires Hashable<std::ranges::range_value_t<T>>
  struct hash<T> {
    std::size_t operator()(const T &x) const noexcept {
      return boost::hash_range(std::begin(x), std::end(x));
    }
  };
  template <typename T>
    requires std::is_aggregate_v<T> && (!std::ranges::input_range<T>)
  struct hash<T> {
    std::size_t operator()(const T &x) const noexcept {
      size_t seed = 0;

      boost::pfr::for_each_field(x, [&seed](auto &&member) {
        boost::hash_combine(
            seed, std::hash<std::remove_cvref_t<decltype(member)>>()(member));
      });

      return seed;
    }
  };
} // namespace std
