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
#include <string>
#include <unordered_map>

#include <boost/bimap.hpp>

#include "alphabet.hpp"
#include "exception.hpp"

namespace cyy::algorithm {
  template <typename T>
  concept MapType =
      std::same_as<
          T, std::map<typename T::key_type, typename T::mapped_type,
                      typename T::key_compare, typename T::allocator_type>> ||
      std::same_as<
          T, std::unordered_map<typename T::key_type, typename T::mapped_type,
                                typename T::hasher, typename T::key_equal,
                                typename T::allocator_type>>;

  template <typename data_type = std::string>
  class map_alphabet final : public ALPHABET {
  public:
    template <MapType T>
    map_alphabet(const T &symbol_map_, std::string_view name_)
        : ALPHABET(name_) {
      if (symbol_map_.empty()) {
        throw exception::empty_alphabet("symbol map is empty");
      }
      for (auto const &[symbol, data] : symbol_map_) {
        symbol_map.insert({symbol, data});
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
    std::string __to_string(symbol_type symbol) const override {
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
