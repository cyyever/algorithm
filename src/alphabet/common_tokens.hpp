/*!
 * \file lang.cpp
 *
 * \author cyy
 * \date 2018-03-03
 */
#pragma once

#include "range_alphabet.hpp"

namespace cyy::algorithm {

  enum class common_token : symbol_type {
    ascii_char = 256,
    escape_sequence,
    digit,
    number,
    whitespace,
    id,
    INT,
    FLOAT,
    record,
    CLASS,
    _end,
  };

  class common_tokens final : public interval_alphabet {
  public:
    common_tokens()
        : interval_alphabet(0, static_cast<symbol_type>(common_token::_end) - 1,
                            "common_tokens") {}

  private:
    std::string _to_string(symbol_type symbol) const override;
    bool support_ASCII_escape_sequence() const noexcept override {
      return true;
    }
  };

} // namespace cyy::algorithm
