/*!
 * \file sub_alphabet.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include "set_alphabet.hpp"

namespace cyy::algorithm {
  class sub_alphabet final : public set_alphabet {
  public:
    sub_alphabet(ALPHABET_ptr parent_alphabet_,
                 const symbol_set_type &symbol_set, std::string_view name_ = "")
        : set_alphabet(symbol_set, std::string("sub_alphabet_of_") +
                                       parent_alphabet_->get_name()),
          parent_alphabet(parent_alphabet_) {
      if (!name_.empty()) {
        set_name(name_);
      }
    }

  private:
    std::string __to_string(symbol_type symbol) const override {
      return parent_alphabet->to_string(symbol);
    }

  private:
    ALPHABET_ptr parent_alphabet;
  };

} // namespace cyy::algorithm
