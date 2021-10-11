/*!
 * \file set_alphabet.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include <algorithm>
#include <set>
#include <vector>

#include "alphabet.hpp"
#include "exception.hpp"

namespace cyy::algorithm {

  class set_alphabet : public ALPHABET {
  public:
    set_alphabet(const symbol_set_type &symbol_set, std::string_view name_)
        : ALPHABET(name_), symbols(symbol_set.begin(), symbol_set.end()) {
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

  private:
    std::vector<symbol_type> symbols{};
  };

} // namespace cyy::algorithm
