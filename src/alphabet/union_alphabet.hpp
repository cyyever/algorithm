/*!
 * \file union_alphabet.hpp
 *
 */
#pragma once

#include <memory>

#include "alphabet.hpp"

namespace cyy::algorithm {

  class union_alphabet : public ALPHABET {
  public:
    union_alphabet(ALPHABET_ptr alphabet1_, ALPHABET_ptr alphabet2_,
                   std::string_view name_ = "")
        : ALPHABET("placeholder"), alphabet1{alphabet1_}, alphabet2{
                                                              alphabet2_} {
      auto alphabet1_max = alphabet1->get_max_symbol();
      auto alphabet2_min = alphabet2->get_min_symbol();
      if (alphabet1_max >= alphabet2_min) {
        throw cyy::algorithm::exception::invalid_alphabet(
            "alphabet1 is not less thant alphabet2");
      }

      if (name_.empty()) {
        set_name(alphabet1->get_name() + "_union_" + alphabet2->get_name());
      }
    }

    bool contain(symbol_type s) const noexcept override {
      return alphabet1->contain(s) || alphabet2->contain(s);
    }

    size_t size() const noexcept override {
      return alphabet1->size() + alphabet2->size();
    }

    symbol_type get_symbol(size_t index) const noexcept override {
      auto alphabet1_size = alphabet1->size();
      if (index < alphabet1_size) {
        return alphabet1->get_symbol(index);
      }
      return alphabet2->get_symbol(index - alphabet1_size);
    }
    bool support_ASCII_escape_sequence() const override {
      return alphabet1->support_ASCII_escape_sequence() ||
             alphabet2->support_ASCII_escape_sequence();
    }

    std::string MMA_draw(symbol_type symbol) const override {
      if (alphabet1->contain(symbol)) {
        return alphabet1->MMA_draw(symbol);
      }
      return alphabet2->MMA_draw(symbol);
    }

  private:
    std::string __to_string(symbol_type symbol) const override {
      if (alphabet1->contain(symbol)) {
        return alphabet1->to_string(symbol);
      }
      return alphabet2->to_string(symbol);
    }

  private:
    ALPHABET_ptr alphabet1;
    ALPHABET_ptr alphabet2;
  };
} // namespace cyy::algorithm
