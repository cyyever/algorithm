/*!
 * \file map_alphabet.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include <format>
#include <map>
#include <ranges>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <boost/bimap.hpp>

#include "alphabet.hpp"
#include "exception.hpp"

namespace cyy::algorithm {
  template <typename data_type = std::string>
  class map_alphabet final : public ALPHABET {
  public:
    template <std::ranges::input_range T>
      requires std::is_same_v<std::ranges::range_value_t<T>,
                              std::pair<const symbol_type, data_type>>
    map_alphabet(T &&symbol_map_, std::string_view name_) : ALPHABET(name_) {
      if (symbol_map_.empty()) {
        throw exception::empty_alphabet("symbol map is empty");
      }
      for (auto &&[symbol, data] : std::forward<T>(symbol_map_)) {
        symbol_map.insert({symbol, std::forward<data_type>(data)});
      }
    }
    bool contain(symbol_type s) const noexcept override {
      return symbol_map.left.find(s) != symbol_map.left.end();
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

    data_type get_data(symbol_type symbol) const {
      return symbol_map.left.at(symbol);
    }
    symbol_type get_symbol(const data_type &data) const {
      return symbol_map.right.at(data);
    }

  private:
    std::string _to_string(symbol_type symbol) const override {
      return std::format("{}", get_data(symbol));
    }
    symbol_type get_symbol(size_t index) const noexcept override {
      auto it = symbol_map.left.begin();
      std::advance(it, index);
      return it->first;
    }

    boost::bimap<symbol_type, data_type> symbol_map;
  };

} // namespace cyy::algorithm
