/*!
 * \file union_alphabet.hpp
 *
 */
#pragma once

#include "alphabet.hpp"

namespace cyy::algorithm {

  class endmarked_alphabet final : public ALPHABET {
  public:
    endmarked_alphabet(ALPHABET_ptr alphabet_)
        : ALPHABET("placeholder"), alphabet{std::move(alphabet_)} {
      if (alphabet->contain(ALPHABET::endmarker)) {
        has_endmarker = true;
        set_name(alphabet->get_name());
      } else {
        set_name(std::string("endmarked_") + alphabet->get_name());
      }
    }

    bool contain(symbol_type s) const noexcept override {
      return alphabet->contain(s) || s == ALPHABET::endmarker;
    }

    std::size_t size() const noexcept override {
      std::size_t real_size = 0;
      if (!has_endmarker) {
        real_size++;
      }

      real_size += alphabet->size();
      return real_size;
    }
    bool support_ASCII_escape_sequence() const noexcept override {
      return alphabet->support_ASCII_escape_sequence();
    }
    auto original_alphabet() const noexcept { return alphabet; }

  private:
    std::string _to_string(symbol_type symbol) const override {
      return alphabet->to_string(symbol);
    }

    symbol_type get_symbol(std::size_t index) const override {
      if (index + 1 == size()) {
        return ALPHABET::endmarker;
      }
      return alphabet->get_symbol(index);
    }

    ALPHABET_ptr alphabet;
    bool has_endmarker{false};
  };
} // namespace cyy::algorithm
