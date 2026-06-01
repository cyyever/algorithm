/*!
 * \file alphabet.hpp
 *
 * \author cyy
 * \date 2018-03-03
 */

#pragma once

#include "symbol.hpp"

#include <concepts>
#include <cstddef>
#include <format>
#include <functional>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace cyy::algorithm {

  // Pairs a symbol with the alphabet that knows how to render it, so a single
  // symbol can be formatted natively with std::format.
  struct alphabet_symbol;

  class ALPHABET {

  public:
    explicit ALPHABET(std::string_view name_) { set_name(name_); }
    ALPHABET(const ALPHABET &) = default;
    ALPHABET &operator=(const ALPHABET &) = default;

    ALPHABET(ALPHABET &&) noexcept = default;
    ALPHABET &operator=(ALPHABET &&) noexcept = default;
    virtual ~ALPHABET() = default;
    bool operator==(const ALPHABET &rhs) const noexcept {
      return name == rhs.name;
    }

    auto get_view() const noexcept {
      return std::ranges::views::iota(static_cast<std::size_t>(0), size()) |
             std::ranges::views::transform(
                 [this](auto idx) { return get_symbol(idx); });
    }

    symbol_type get_min_symbol() const { return get_symbol(0); }
    symbol_type get_max_symbol() const;
    bool contain_alphabet(const ALPHABET &subset) const;

    virtual bool contain(symbol_type s) const noexcept = 0;
    virtual std::size_t size() const noexcept = 0;
    std::string to_string(symbol_type symbol) const;

    const std::string &get_name() const noexcept { return name; }

    virtual bool support_ASCII_escape_sequence() const noexcept {
      return false;
    }

    virtual symbol_type get_symbol(std::size_t index) const = 0;
    void set_MMA_draw_fun(
        std::function<std::string(const ALPHABET &, symbol_type)> fun) {
      MMA_draw_fun_ptr = std::move(fun);
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
    void set_name(std::string_view name_);

  private:
    friend struct std::formatter<alphabet_symbol>;
    virtual std::string _to_string(symbol_type symbol) const {
      return std::format("'{}'", static_cast<char>(symbol));
    }

    static std::unordered_map<std::string, std::shared_ptr<ALPHABET>> &
    get_factory();

    std::function<std::string(const ALPHABET &, symbol_type)> MMA_draw_fun_ptr;
    std::string name;
  };

  struct alphabet_symbol {
    const ALPHABET &alphabet;
    symbol_type symbol;
  };

  class ALPHABET_ptr : public std::shared_ptr<ALPHABET> {
  public:
    using std::shared_ptr<ALPHABET>::shared_ptr;
    template <typename T>
    ALPHABET_ptr(const T &strv) : ALPHABET_ptr(ALPHABET::get(strv)) {}
    // NOLINTNEXTLINE
    ALPHABET_ptr(std::shared_ptr<ALPHABET> ptr) noexcept
        : shared_ptr(std::move(ptr)) {}
  };

  inline auto endmarked_symbol_string(symbol_string_view str) noexcept {
    auto size = str.size() + 1;
    return std::ranges::views::iota(static_cast<std::size_t>(0), size) |
           std::ranges::views::transform([str, size](auto idx) {
             if (idx + 1 == size) {
               return ALPHABET::endmarker;
             }
             return str[idx];
           });
  }

} // namespace cyy::algorithm

template <typename T>
  requires std::derived_from<T, cyy::algorithm::ALPHABET>
struct std::formatter<T> {
  // Parses format specifications (only the empty spec "{}" is accepted).
  template <class ParseContext> constexpr auto parse(ParseContext &ctx) {
    auto it = ctx.begin();
    if (it != ctx.end() && *it != '}') {
      throw std::format_error("invalid format");
    }
    return it;
  }

  template <class FormatContext>
  auto format(const cyy::algorithm::ALPHABET &alphabet,
              FormatContext &ctx) const {
    std::format_to(ctx.out(), "alphabet {}:", alphabet.get_name());
    for (auto const symbol : alphabet.get_view()) {
      std::format_to(ctx.out(), " {}", alphabet.to_string(symbol));
    }
    return ctx.out();
  }
};

template <> struct std::formatter<cyy::algorithm::alphabet_symbol> {
  template <class ParseContext> constexpr auto parse(ParseContext &ctx) {
    auto it = ctx.begin();
    if (it != ctx.end() && *it != '}') {
      throw std::format_error("invalid format");
    }
    return it;
  }

  template <class FormatContext>
  auto format(const cyy::algorithm::alphabet_symbol &s,
              FormatContext &ctx) const {
    const auto &alphabet = s.alphabet;
    if (alphabet.contain(s.symbol)) {
      return std::format_to(ctx.out(), "{}", alphabet._to_string(s.symbol));
    }
    if (s.symbol == cyy::algorithm::ALPHABET::endmarker) {
      return std::format_to(ctx.out(), "$");
    }
    if (s.symbol == cyy::algorithm::ALPHABET::blank_symbol) {
      return std::format_to(ctx.out(), "blank");
    }
    return std::format_to(ctx.out(), "(unknown symbol)");
  }
};

namespace cyy::algorithm {
  // Defined here, after std::formatter<alphabet_symbol> is visible, so the
  // std::format call below picks up the explicit specialization.
  inline std::string ALPHABET::to_string(symbol_type symbol) const {
    return std::format("{}", alphabet_symbol{.alphabet = *this, .symbol = symbol});
  }
} // namespace cyy::algorithm
