/*!
 * \file lang.cpp
 *
 * \author cyy
 * \date 2018-03-03
 */
#pragma once

#include "set_alphabet.hpp"

namespace cyy::algorithm {

  class number_set_alphabet final : public set_alphabet {
  public:
    using set_alphabet::set_alphabet;

  private:
    std::string __to_string(symbol_type symbol) const override {
      return std::to_string(static_cast<uint64_t>(symbol));
    }
  };

} // namespace cyy::algorithm
