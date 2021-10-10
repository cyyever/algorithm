/*!
 * \file map_alphabet.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include <map>

#include "alphabet.hpp"
#include "exception.hpp"

namespace cyy::algorithm {
  class map_alphabet final : public ALPHABET {
  public:
    map_alphabet(std::map<symbol_type, std::string> symbol_map_,
                 std::string_view name_)
        : ALPHABET(name_), symbol_map(std::move(symbol_map_)) {
      if (symbol_map.empty()) {
        throw exception::empty_alphabet("symbol map is empty");
      }
      for (auto const &[symbol, data] : symbol_map) {
        auto has_inserted = reverse_symbol_map.try_emplace(data, symbol).second;
        if (!has_inserted) {
          throw exception::invalid_alphabet(std::string("same data ") + data +
                                            " in several symbols");
        }
      }
    }
    bool contain(symbol_type s) const noexcept override {
      return symbol_map.contains(s);
    }
    size_t size() const noexcept override { return symbol_map.size(); }

    std::string MMA_draw_set() const {
      std::string cmd = "{";
      for (auto const &[s, _] : symbol_map) {
        cmd += MMA_draw(s);
        cmd.push_back(',');
      }
      cmd.back() = '}';
      return cmd;
    }

    std::string get_data(symbol_type symbol) const {
      return symbol_map.at(symbol);
    }
    symbol_type get_symbol(const std::string &data) const {
      return reverse_symbol_map.at(data);
    }

  private:
    std::string __to_string(symbol_type symbol) const override {
      return get_data(symbol);
    }
    symbol_type get_symbol(size_t index) const noexcept override {
      auto it = symbol_map.begin();
      std::advance(it, index);
      return it->first;
    }

  private:
    std::map<symbol_type, std::string> symbol_map;
    std::map<std::string, symbol_type> reverse_symbol_map;
  };

} // namespace cyy::algorithm
