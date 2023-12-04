/*!
 * \file sub_alphabet.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include "range_alphabet.hpp"

namespace cyy::algorithm {
  class sub_alphabet final : public set_alphabet {
  public:
    template <std::ranges::input_range T>
    sub_alphabet(ALPHABET_ptr parent_alphabet_, T symbol_set)
        : set_alphabet(std::forward<T>(symbol_set),
                       std::string("sub_alphabet_of_") +
                           parent_alphabet_->get_name()),
          parent_alphabet(std::move(parent_alphabet_)) {}

  private:
    std::string _to_string(symbol_type symbol) const override {
      return parent_alphabet->to_string(symbol);
    }

    ALPHABET_ptr parent_alphabet;
  };

} // namespace cyy::algorithm
