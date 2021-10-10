/*!
 * \file alphabet.hpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

#include <set>
#include <string>
#include <string_view>

namespace cyy::computation {

  using symbol_type = char32_t;
  using symbol_set_type = std::set<symbol_type>;
  using symbol_string = std::basic_string<symbol_type>;
  using symbol_string_view = std::basic_string_view<symbol_type>;
} // namespace cyy::computation
