/*!
 * \file union_alphabet.hpp
 *
 */
#pragma once

#include "alphabet.hpp"

namespace cyy::algorithm {

  class alphabet_with_blank_symbol final : public ALPHABET {
  public:
    explicit alphabet_with_blank_symbol(ALPHABET_ptr alphabet_)
        : ALPHABET("placeholder"), alphabet{std::move(alphabet_)} {
      if (alphabet->contain(ALPHABET::blank_symbol)) {
        has_blank_symbol = true;
        set_name(alphabet->get_name());
      } else {
        set_name(alphabet->get_name() + "_with_blank_symbol");
      }
      has_endmarker = alphabet->contain(ALPHABET::endmarker);
    }

    bool contain(symbol_type s) const noexcept override {
      return alphabet->contain(s) || s == ALPHABET::blank_symbol;
    }

    size_t size() const noexcept override {
      size_t real_size = 0;
      if (!has_blank_symbol) {
        real_size++;
      }

      real_size += alphabet->size();
      return real_size;
    }
    bool support_ASCII_escape_sequence() const noexcept override {
      return alphabet->support_ASCII_escape_sequence();
    }
    auto original_alphabet() const { return alphabet; }

  private:
    std::string _to_string(symbol_type symbol) const override {
      return alphabet->to_string(symbol);
    }

    symbol_type get_symbol(size_t index) const noexcept override {
      if (has_blank_symbol) {
        return alphabet->get_symbol(index);
      }
      if (index + 1 == size()) {
        if (has_endmarker) {
          return ALPHABET::endmarker;
        }
        return ALPHABET::blank_symbol;
      }
      if (index + 2 == size()) {
        if (has_endmarker) {
          return ALPHABET::blank_symbol;
        }
      }
      return alphabet->get_symbol(index);
    }

    ALPHABET_ptr alphabet;
    bool has_blank_symbol{false};
    bool has_endmarker{false};
  };
} // namespace cyy::algorithm
