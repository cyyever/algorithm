/*!
 * \file map_alphabet.hpp
 *
 * \brief
 * \author cyy
 * \date 2018-03-31
 */
#pragma once

#include "alphabet.hpp"
#include "exception.hpp"
#include "pool.hpp"

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
        auto data_id = data_pool.add_data(data);
        symbol_to_data_id.emplace(symbol, data_id);
      }
    }
    bool contain(symbol_type s) const noexcept override {
      return symbol_to_data_id.contains(s);
    }
    std::size_t size() const noexcept override {
      return symbol_to_data_id.size();
    }

    std::string MMA_draw_set() const {
      std::string cmd = "{";
      for (auto const &[s, _] : symbol_to_data_id) {
        cmd += MMA_draw(s);
        cmd.push_back(',');
      }
      cmd.back() = '}';
      return cmd;
    }

    data_type get_data(symbol_type symbol) const {
      auto data_id = symbol_to_data_id.at(symbol);
      return data_pool.get_data(data_id);
    }
    symbol_type get_symbol(const data_type &data) const {
      auto data_id = data_pool.get_data_id(data);
      return data_id_to_symbol.at(data_id);
    }

  private:
    std::string _to_string(symbol_type symbol) const override {
      return std::format("{}", get_data(symbol));
    }
    symbol_type get_symbol(std::size_t index) const noexcept override {
      auto it = symbol_to_data_id.begin();
      std::advance(it, index);
      return it->first;
    }

    cyy::algorithm::object_pool<data_type> data_pool;
    std::unordered_map<symbol_type,
                       typename decltype(data_pool)::element_id_type>
        symbol_to_data_id;
    std::unordered_map<typename decltype(data_pool)::element_id_type,
                       symbol_type>
        data_id_to_symbol;
  };

} // namespace cyy::algorithm
