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

class ASCII final : public range_alphabet {
public:
  ASCII() : range_alphabet(0, 127, "ASCII") {}
  bool support_ASCII_escape_sequence() const override { return true; }
};

class printable_ASCII final : public range_alphabet {
public:
  printable_ASCII() : range_alphabet(32, 126, "printable-ASCII") {}
};
} // namespace cyy::algorithm
