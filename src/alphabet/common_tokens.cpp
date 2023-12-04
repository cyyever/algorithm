/*!
 * \file lang.cpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#include "common_tokens.hpp"

#include "ascii.hpp"

namespace cyy::algorithm {
  std::string common_tokens::_to_string(symbol_type symbol) const {
    switch (static_cast<common_token>(symbol)) {
      case common_token::ascii_char:
        return "\"ascii char\"";
      case common_token::escape_sequence:
        return "\"escape sequence\"";
      case common_token::id:
        return "\"id\"";
      case common_token::whitespace:
        return "\"whitespace\"";
      case common_token::digit:
        return "\"digit\"";
      case common_token::number:
        return "\"number\"";
      case common_token::INT:
        return "\"int\"";
      case common_token::FLOAT:
        return "\"double\"";
      case common_token::record:
        return "\"record\"";
      case common_token::CLASS:
        return "\"class\"";
      case common_token::_end:
        return "";
    }
    return ASCII().to_string(symbol);
  }
} // namespace cyy::algorithm
