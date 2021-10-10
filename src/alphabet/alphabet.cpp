/*!
 * \file lang.cpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#include "alphabet.hpp"

#include <limits>
#include <ranges>
#include <string_view>

#include "../exception.hpp"
#include "ascii.hpp"
#include "common_tokens.hpp"
#include "endmarked_alphabet.hpp"
#include "set_alphabet.hpp"

namespace cyy::computation {
  symbol_type ALPHABET::get_max_symbol() const {
    if (contain(endmarker)) {
      return get_symbol(size() - 2);
    }
    return get_symbol(size() - 1);
  }
  bool ALPHABET::contain(const ALPHABET &subset) const {
    for (auto s : subset.get_view()) {
      if (!contain(s)) {
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<ALPHABET> ALPHABET::get(std::string_view name,
                                          bool endmarked) {
    register_factory();
    auto it = factory.find(std::string(name));
    if (it == factory.end()) {
      throw exception::unexisted_alphabet(std::string(name));
    }
    it->second->name = name;
    auto alphabet = it->second;
    if (endmarked) {
      alphabet = std::make_shared<endmarked_alphabet>(alphabet);
    }
    return alphabet;
  }

  void ALPHABET::set(const std::shared_ptr<ALPHABET> &alphabet) {
    register_factory();
    factory[alphabet->get_name()] = alphabet;
  }

  void ALPHABET::register_factory() {
    if (!factory.empty()) {
      return;
    }

    for (auto alphabet : std::initializer_list<ALPHABET_ptr>{
             std::make_shared<common_tokens>(), std::make_shared<ASCII>(),
             std::make_shared<printable_ASCII>(),
             std::make_shared<set_alphabet>(symbol_set_type{'a', 'b'},
                                            "ab_set"),
             std::make_shared<set_alphabet>(symbol_set_type{'(', ')'},
                                            "parentheses"),

             std::make_shared<set_alphabet>(symbol_set_type{'0'}, "0_set"),
             std::make_shared<set_alphabet>(symbol_set_type{'0', '1'},
                                            "01_set"),
             std::make_shared<set_alphabet>(symbol_set_type{'0', '1', '#'},
                                            "01#_set"),
             std::make_shared<set_alphabet>(symbol_set_type{'0', '1', '#', 'x'},
                                            "01x#_set")}) {
      factory.emplace(alphabet->get_name(), alphabet);
    }
  }
  std::string ALPHABET::MMA_draw(symbol_type symbol) const {
    if (MMA_draw_fun_ptr) {
      return (*MMA_draw_fun_ptr)(*this, symbol);
    }
    auto cmd = to_string(symbol);
    if (cmd[0] == '\'') {
      cmd[0] = '\"';
      cmd.back() = '\"';
    }
    cmd = std::string("Style[") + cmd + ",Bold,Purple]";
    return cmd;
  }
} // namespace cyy::computation
