/*!
 * \file symbol.hpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

import std;

namespace cyy::algorithm {

  using symbol_type = char32_t;
  using symbol_set_type = std::set<symbol_type>;
  using symbol_init_list = std::initializer_list<symbol_type>;
  using symbol_string = std::basic_string<symbol_type>;
  using symbol_string_view = std::basic_string_view<symbol_type>;
} // namespace cyy::algorithm
