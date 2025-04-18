/*!
 * \file lang.cpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#include "alphabet.hpp"

#include "ascii.hpp"
#include "common_tokens.hpp"
#include "endmarked_alphabet.hpp"
#include "exception.hpp"
import std;

namespace cyy::algorithm {

  void ALPHABET::set_name(std::string_view name_) {
    if (name_.empty()) {
      throw exception::empty_alphabet_name("");
    }
    name = name_;
  }

  symbol_type ALPHABET::get_max_symbol() const {
    if (contain(endmarker)) {
      return get_symbol(size() - 2);
    }
    return get_symbol(size() - 1);
  }
  bool ALPHABET::contain_alphabet(const ALPHABET &subset) const {
    return std::ranges::all_of(subset.get_view(),
                               [this](const auto s) { return contain(s); });
  }

  std::shared_ptr<ALPHABET> ALPHABET::get(std::string_view name,
                                          bool endmarked) {
    const auto &factory = get_factory();
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
    auto &factory = get_factory();
    factory[alphabet->get_name()] = alphabet;
  }

  std::unordered_map<std::string, std::shared_ptr<ALPHABET>> &
  ALPHABET::get_factory() {
    static std::unordered_map<std::string, std::shared_ptr<ALPHABET>> factory;
    if (factory.empty()) {
      for (const auto &alphabet : std::initializer_list<ALPHABET_ptr>{
               std::make_shared<common_tokens>(), std::make_shared<ASCII>(),
               std::make_shared<printable_ASCII>(),
               std::make_shared<set_alphabet>(symbol_init_list{'a', 'b'},
                                              "ab_set"),
               std::make_shared<set_alphabet>(symbol_init_list{'(', ')'},
                                              "parentheses"),

               std::make_shared<set_alphabet>(symbol_init_list{'0'}, "0_set"),
               std::make_shared<set_alphabet>(symbol_init_list{'0', '1'},
                                              "01_set"),
               std::make_shared<set_alphabet>(symbol_init_list{'0', '1', '#'},
                                              "01#_set"),
               std::make_shared<set_alphabet>(
                   symbol_init_list{'0', '1', '#', 'x'}, "01x#_set")}) {
        factory.emplace(alphabet->get_name(), alphabet);
      }
    }

    return factory;
  }
  std::string ALPHABET::MMA_draw(symbol_type symbol) const {
    if (MMA_draw_fun_ptr) {
      return MMA_draw_fun_ptr(*this, symbol);
    }
    auto cmd = to_string(symbol);
    if (cmd[0] == '\'') {
      cmd[0] = '\"';
      cmd.back() = '\"';
    }
    cmd = std::format("Style[{},Bold,Purple]", cmd);
    return cmd;
  }
} // namespace cyy::algorithm
