/*!
 * \file range_alphabet.hpp
 *
 */
#pragma once

#include <algorithm>
#include <ranges>
#include <set>
#include <vector>

#include "alphabet.hpp"

namespace cyy::algorithm {

  class interval_alphabet : public ALPHABET {
  public:
    interval_alphabet(symbol_type min_symbol_, symbol_type max_symbol_,
                      std::string_view name_)
        : ALPHABET(name_), min_symbol{min_symbol_}, max_symbol{max_symbol_} {
      if (max_symbol < min_symbol) {
        throw exception::empty_alphabet("range is empty");
      }
    }

    bool contain(symbol_type s) const noexcept override {
      return s >= min_symbol && s <= max_symbol;
    }

    size_t size() const noexcept override {
      return max_symbol - min_symbol + 1;
    }

  private:
    symbol_type get_symbol(size_t index) const noexcept override {
      return static_cast<symbol_type>(min_symbol + index);
    }

    symbol_type min_symbol;
    symbol_type max_symbol;
  };

  class set_alphabet : public ALPHABET {
  public:
    template <std::ranges::input_range T>
    set_alphabet(T symbols_, std::string_view name_) : ALPHABET(name_) {
      std::set<symbol_type> symbol_set(symbols_.begin(), symbols_.end());
      symbols = decltype(symbols)(symbol_set.begin(), symbol_set.end());

      if (symbols.empty()) {
        throw exception::empty_alphabet("symbol set is empty");
      }
    }

    bool contain(symbol_type s) const noexcept override {
      return std::ranges::binary_search(symbols, s);
    }
    size_t size() const noexcept override { return symbols.size(); }

  private:
    symbol_type get_symbol(size_t index) const noexcept override {
      return symbols[index];
    }

    std::vector<symbol_type> symbols{};
  };

} // namespace cyy::algorithm
