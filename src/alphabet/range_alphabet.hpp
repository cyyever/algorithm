/*!
 * \file range_alphabet.hpp
 *
 */
#pragma once

#include "alphabet.hpp"

namespace cyy::algorithm {

class range_alphabet : public ALPHABET {
public:
  range_alphabet(symbol_type min_symbol_, symbol_type max_symbol_,
                 std::string_view name_)
      : ALPHABET(name_), min_symbol{min_symbol_}, max_symbol{max_symbol_} {
    if (max_symbol < min_symbol) {
      throw exception::empty_alphabet("range is empty");
    }
  }

  bool contain(symbol_type s) const noexcept override {
    return s >= min_symbol && s <= max_symbol;
  }

  size_t size() const noexcept override { return max_symbol - min_symbol + 1; }

private:
  symbol_type get_symbol(size_t index) const noexcept override {
    return static_cast<symbol_type>(min_symbol + index);
  }

private:
  symbol_type min_symbol;
  symbol_type max_symbol;
};
} // namespace cyy::algorithm
