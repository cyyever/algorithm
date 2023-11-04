/*!
 * \file ascii.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include "range_alphabet.hpp"

namespace cyy::algorithm {

  class ASCII final : public interval_alphabet {
  public:
    ASCII() : interval_alphabet(0, 127, "ASCII") {}
    bool support_ASCII_escape_sequence() const override { return true; }
  };

  class printable_ASCII final : public interval_alphabet {
  public:
    printable_ASCII() : interval_alphabet(32, 126, "printable-ASCII") {}
    bool support_ASCII_escape_sequence() const override { return true; }
  };
} // namespace cyy::algorithm
