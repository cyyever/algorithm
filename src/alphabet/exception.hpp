/*!
 * \file exception.hpp
 *
 * \brief
 */

#pragma once

import std;

namespace cyy::algorithm::exception {

  class empty_alphabet_name : public std::invalid_argument {
  public:
    using invalid_argument::invalid_argument;
  };
  class empty_alphabet : public std::invalid_argument {
  public:
    using invalid_argument::invalid_argument;
  };

  class invalid_alphabet : public std::invalid_argument {
  public:
    using invalid_argument::invalid_argument;
  };
  class unexisted_alphabet : public std::invalid_argument {
  public:
    using invalid_argument::invalid_argument;
  };

} // namespace cyy::algorithm::exception
