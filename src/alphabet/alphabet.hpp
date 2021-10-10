/*!
 * \file alphabet.hpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <string_view>
#include <unordered_map>

#include "../exception.hpp"
#include "symbol.hpp"

namespace cyy::computation {

  class ALPHABET {

  public:
    explicit ALPHABET(std::string_view name_) { set_name(name_); }
    ALPHABET(const ALPHABET &) = default;
    ALPHABET &operator=(const ALPHABET &) = default;

    ALPHABET(ALPHABET &&) noexcept = default;
    ALPHABET &operator=(ALPHABET &&) noexcept = default;
    virtual ~ALPHABET() = default;
    bool operator==(const ALPHABET &rhs) const = default;

    auto get_view() const {
      return std::ranges::views::iota(static_cast<size_t>(0), size()) |
             std::ranges::views::transform(
                 [this](auto idx) { return get_symbol(idx); });
    }

    symbol_type get_min_symbol() const { return get_symbol(0); }
    symbol_type get_max_symbol() const;
    bool contain(const ALPHABET &subset) const;

    virtual bool contain(symbol_type s) const = 0;
    virtual size_t size() const = 0;
    std::string to_string(symbol_type symbol) const {
      if (symbol == endmarker) {
        return "$";
      }
      if (symbol == blank_symbol) {
        return "blank";
      }
      if (contain(symbol)) {
        return __to_string(symbol);
      }
      return "(unknown symbol)";
    }

    const std::string &get_name() const { return name; }

    virtual bool support_ASCII_escape_sequence() const { return false; }

    virtual symbol_type get_symbol(size_t index) const = 0;
    void set_MMA_draw_fun(
        std::function<std::string(const ALPHABET &, symbol_type)> fun) {
      MMA_draw_fun_ptr = std::make_shared<decltype(fun)>(fun);
    }
    virtual std::string MMA_draw(symbol_type symbol) const;

    static std::shared_ptr<ALPHABET> get(std::string_view name,
                                         bool endmarked = false);
    static void set(const std::shared_ptr<ALPHABET> &alphabet);
    static constexpr symbol_type endmarker =
        std::numeric_limits<symbol_type>::max() - 1;
    static constexpr symbol_type blank_symbol =
        std::numeric_limits<symbol_type>::max() - 2;

  protected:
    void set_name(std::string_view name_) {
      if (name_.empty()) {
        throw cyy::computation::exception::empty_alphabet_name("");
      }
      name = std::move(name_);
    }

  private:
    virtual std::string __to_string(symbol_type symbol) const {
      return {'\'', static_cast<char>(symbol), '\''};
    }

    static void register_factory();

  private:
    std::shared_ptr<std::function<std::string(const ALPHABET &, symbol_type)>>
        MMA_draw_fun_ptr;
    std::string name;

  private:
    static inline std::unordered_map<std::string, std::shared_ptr<ALPHABET>>
        factory;
  };

  class ALPHABET_ptr : public std::shared_ptr<ALPHABET> {
  public:
    using std::shared_ptr<ALPHABET>::shared_ptr;
    ALPHABET_ptr(const std::shared_ptr<ALPHABET> &ptr) : shared_ptr(ptr) {}
    ALPHABET_ptr(const char *str) : ALPHABET_ptr(ALPHABET::get(str)) {}
    ALPHABET_ptr(std::string_view strv) : ALPHABET_ptr(ALPHABET::get(strv)) {}
    ALPHABET_ptr(const std::string &str) : ALPHABET_ptr(str.c_str()) {}
  };

  inline auto endmarked_symbol_string(symbol_string_view str) {
    auto size = str.size() + 1;
    return std::ranges::views::iota(static_cast<size_t>(0), size) |
           std::ranges::views::transform([str, size](auto idx) {
             if (idx + 1 == size) {
               return ALPHABET::endmarker;
             }
             return str[idx];
           });
  }

} // namespace cyy::computation
