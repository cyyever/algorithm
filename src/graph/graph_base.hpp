/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once
#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <iostream>
#include <iterator>
#include <list>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/bimap.hpp>

#include "pool.hpp"

namespace cyy::algorithm {
  template <typename vertex_type, typename weight_type> struct edge {
    vertex_type first;
    vertex_type second;
    weight_type weight = 1;
    edge(vertex_type first_, vertex_type second_, weight_type weight_ = 1)
        : first(std::move(first_)), second(std::move(second_)),
          weight(std::move(weight_)) {
      if (first == second) {
        throw std::runtime_error("not an edge");
      }
    }
    auto operator<=>(const auto &rhs) const noexcept {
      return weight <=> rhs.weight;
    }
    auto operator==(const auto &rhs) const noexcept {
      return first == rhs.first && second == rhs.second;
    }
    edge reverse() const { return edge(second, first, weight); }
  };

  struct indexed_edge {
    size_t first;
    size_t second;
    indexed_edge(size_t first_, size_t second_)
        : first(first_), second(second_) {
      if (first == second) {
        throw std::runtime_error("not an edge");
      }
    }
    bool operator==(const indexed_edge &rhs) const noexcept = default;
    auto operator<=>(const indexed_edge &rhs) const noexcept = default;
    [[nodiscard]] bool contains(size_t v) const noexcept {
      return first == v || second == v;
    }
    [[nodiscard]] indexed_edge reverse() const { return {second, first}; }
  };
  using path_type = std::vector<size_t>;
} // namespace cyy::algorithm

namespace std {
  template <> struct hash<cyy::algorithm::indexed_edge> {
    size_t operator()(const cyy::algorithm::indexed_edge &x) const noexcept {
      return ::std::hash<size_t>()(x.first) ^ ::std::hash<size_t>()(x.second);
    }
  };
  template <typename vertex_type, typename weight_type>
  // NOLINTNEXTLINE
  struct hash<cyy::algorithm::edge<vertex_type, weight_type>> {
    size_t operator()(const cyy::algorithm::edge<vertex_type, weight_type> &x)
        const noexcept {
      return ::std::hash<vertex_type>()(x.first) ^
             ::std::hash<vertex_type>()(x.second);
    }
  };
} // namespace std

namespace cyy::algorithm {
  template <typename vertexType, bool directed, typename weightType>
  class graph_base {
  public:
    using edge_type = edge<vertexType, weightType>;
    using vertex_type = vertexType;
    using weight_type = weightType;
    using adjacent_matrix_type = std::vector<std::vector<weight_type>>;
    using vertices_type = object_pool<vertex_type, true>;
    static constexpr bool is_directed = directed;
    graph_base() = default;
    template <std::ranges::input_range U>
      requires std::same_as<edge_type, std::ranges::range_value_t<U>>
    explicit graph_base(U edges) {
      for (auto const &edge : edges) {
        add_edge(edge);
      }
    }
    void set_vertex_indices(vertices_type new_vertices) {
      assert(weighted_adjacent_list.empty());
      vertex_pool = std::move(new_vertices);
    }
    const auto &get_vertex_pool() const { return vertex_pool; }
    [[nodiscard]] bool empty() const { return vertex_pool.empty(); }

    void print_edges(std::ostream &os) const {
      for (auto const &e : foreach_edge()) {
        os << e.first << " -> " << e.second << std::endl;
      }
    }

    [[nodiscard]] constexpr bool has_continuous_vertices() const { return true; }

    auto foreach_edge_with_weight() const noexcept {
      if constexpr (directed) {
        return std::views::join(std::views::transform(
            weighted_adjacent_list, [](const auto &p) noexcept {
              return std::ranges::views::transform(
                  p.second, [&p](auto const &t) {
                    return std::pair(indexed_edge{p.first, t.first}, t.second);
                  });
            }));
      } else {
        return std::views::join(std::views::transform(
            weighted_adjacent_list, [](const auto &p) noexcept {
              return std::ranges::views::filter(
                         p.second,
                         [&p](const std::pair<size_t, weight_type> &e) -> bool {
                           return e.first > p.first;
                         }) |
                     std::ranges::views::transform([&p](auto const &t) {
                       return std::pair(indexed_edge{p.first, t.first},
                                        t.second);
                     });
            }));
      }
    }

    auto foreach_edge() const noexcept {
      return foreach_edge_with_weight() | std::views::keys;
    }

    size_t add_dummy_vertex() {
      assert(!empty());
      if constexpr (std::is_same_v<vertex_type, std::string>) {
        static constexpr auto artificial_vertex_name = "___dummy_vertex";
        return add_vertex(artificial_vertex_name);
      } else {
        static_assert(std::is_integral_v<vertex_type>);
        return add_vertex(get_vertex_number());
      }
    }

    size_t add_vertex(vertex_type vertex) {
      return vertex_pool.add_data(vertex);
    }

    edge_type get_edge(const indexed_edge &edge) const {
      for (auto const &to_vertice : get_adjacent_list(edge.first)) {
        if (to_vertice.first == edge.second) {
          return {get_vertex(edge.first), get_vertex(edge.second),
                  to_vertice.second};
        }
      }
      throw std::runtime_error("no edge");
    }

    indexed_edge get_edge(const edge_type &edge) const {
      return {get_vertex_index(edge.first), get_vertex_index(edge.second)};
    }
    [[nodiscard]] bool has_edge(const indexed_edge &e) const {
      auto const &l = get_adjacent_list(e.first);
      return std::ranges::find_if(l, [&e](auto const &p) {
               return p.first == e.second;
             }) != l.end();
    }
    template <bool minimum = true>
    auto get_extreme_weight(const path_type &path) {
      weight_type extreme_weight{};
      for (size_t i = 0; i + 1 < path.size(); i++) {
        indexed_edge e{path[i], path[i + 1]};
        if (i == 0) {
          extreme_weight = get_edge(e).weight;
        } else {
          if constexpr (minimum) {
            extreme_weight = std::min(extreme_weight, get_edge(e).weight);
          } else {
            extreme_weight = std::max(extreme_weight, get_edge(e).weight);
          }
        }
      }
      return extreme_weight;
    }

    bool has_vertex(const vertex_type &vertex) const {
      return vertex_pool.contains(vertex);
    }
    [[nodiscard]] bool has_vertex_index(size_t vertex_index) const {
      return vertex_pool.contains_data_id(vertex_index);
    }
    // get a path from u to v
    [[nodiscard]] path_type get_path(size_t u, size_t v) const {
      path_type path{u};
      while (u != v) {
        auto const &l = this->get_adjacent_list(u);
        if (l.empty()) {
          return {};
        }
        u = l.begin()->first;
        path.push_back(u);
      }
      assert(path.back() == v);
      return path;
    }
    // get a path from u to v
    [[nodiscard]] bool has_path(size_t u, size_t v) const {
      return !get_path(u, v).empty();
    }
    void add_edge(const edge_type &e) {
      add_directed_edge(e);
      if constexpr (!directed) {
        add_directed_edge(e.reverse());
      }
    }
    void clear_edges() { weighted_adjacent_list.clear(); }
    void remove_vertex(size_t vertex_index) {
      weighted_adjacent_list.erase(vertex_index);
      for (auto &[_, to_vertices] : weighted_adjacent_list) {
        to_vertices.remove_if(
            [vertex_index](auto const &a) { return a.first == vertex_index; });
      }
    }
    bool remove_edge(const edge_type &e) {
      return remove_edge(
          {get_vertex_index(e.first), get_vertex_index(e.second)});
    }

    bool remove_edge(const indexed_edge &e) {
      if (!remove_directed_edge(e)) {
        return false;
      }
      if constexpr (!directed) {
        remove_directed_edge(e.reverse());
      }
      return true;
    }

    auto const &get_adjacent_list() const { return weighted_adjacent_list; }
    auto const &get_adjacent_list(size_t vertex_index) const {
      auto it = weighted_adjacent_list.find(vertex_index);
      if (it != weighted_adjacent_list.end()) {
        return it->second;
      }
      return empty_adjacent_list;
    }
    adjacent_matrix_type get_adjacent_matrix() const {
      adjacent_matrix_type adjacent_matrix;
      auto vertex_num = get_vertex_number();
      adjacent_matrix.reserve(vertex_num);
      for (size_t i = 0; i < vertex_num; i++) {
        adjacent_matrix.emplace_back(vertex_num, 0);
      }
      for (auto const &[e, weight] : foreach_edge_with_weight()) {
        adjacent_matrix[e.first][e.second] = weight;
        if constexpr (!directed) {
          adjacent_matrix[e.second][e.first] = weight;
        }
      }
      return adjacent_matrix;
    }

    auto get_vertex_indices() const noexcept {
      return std::views::transform(vertex_pool.foreach_data(),
                                   [](auto const &it) { return it.second; });
    }

    auto get_vertices_and_indices() const {
      return std::views::transform(
          vertex_pool.foreach_data(), [](auto const &it) {
            return std::pair<const vertex_type &, size_t>{it.first, it.second};
          });
    }
    [[nodiscard]] size_t get_vertex_number() const {
      return vertex_pool.size();
    }
    [[nodiscard]] size_t get_edge_number() const {
      return static_cast<size_t>(std::ranges::distance(foreach_edge()));
    }
    const vertex_type &get_vertex(size_t index) const {
      return vertex_pool.get_data(index);
    }
    size_t get_vertex_index(const vertex_type &vertex) const {
      return vertex_pool.get_data_id(vertex);
    }
    void set_all_weights(weight_type new_weight) {
      for (auto &[_, to_vertices] : weighted_adjacent_list) {
        for (auto &to_vertice : to_vertices) {
          to_vertice.second = new_weight;
        }
      }
    }
    auto get_weight(const indexed_edge &edge) const {
      return get_edge(edge).weight;
    }

    void set_weight(const indexed_edge &edge, weight_type new_weight) {
      auto &adjacent_list = weighted_adjacent_list.at(edge.first);
      auto it =
          std::ranges::find_if(adjacent_list, [&edge](auto const &to_vertice) {
            return to_vertice.first == edge.second;
          });
      if (it == adjacent_list.end()) {
        throw std::runtime_error("no edge found");
      }
      it->second = new_weight;
    }

    // breadth first search in g from s
    void breadth_first_search(
        size_t s,
        std::function<bool(size_t, size_t, weight_type)> edge_fun) const {
      assert(has_vertex_index(s));
      std::vector<bool> discovered(get_vertex_number(), false);
      discovered[s] = true;
      std::list<size_t> queue{s};

      while (!queue.empty()) {
        auto u = queue.front();
        queue.pop_front();
        for (auto const &neighbor : get_adjacent_list(u)) {
          auto const &[v, weight] = neighbor;
          if (!discovered[v]) {
            discovered[v] = true;
            queue.push_back(v);
            if (edge_fun(u, v, weight)) {
              return;
            }
          }
        }
      }
    }
    // depth first search in g from s

    void recursive_depth_first_search(
        size_t s,
        const std::function<bool(size_t, size_t)> &after_edge_fun) const {
      assert(has_vertex_index(s));

      std::vector<bool> explored(get_vertex_number(), false);
      auto search_fun = [&](auto &&self, size_t u) {
        if (explored[u]) {
          return;
        }
        explored[u] = true;
        for (auto const &neighbor : get_adjacent_list(u)) {
          self(self, neighbor.first);
          if (after_edge_fun(u, neighbor.first)) {
            break;
          }
        }
      };
      search_fun(search_fun, s);
    }

    // depth first search in g from s
    void depth_first_search(
        size_t s,
        std::function<void(size_t, size_t, weight_type)> edge_fun) const {
      assert(has_vertex_index(s));
      std::vector<bool> explored(get_vertex_number(), false);
      std::vector<std::pair<size_t, weight_type>> stack{{s, 0}};
      std::vector<size_t> parent(get_vertex_number(), 0);

      while (!stack.empty()) {
        auto const [u, weight] = stack.back();
        stack.pop_back();
        if (explored[u]) {
          continue;
        }
        explored[u] = true;
        if (u != s) {
          edge_fun(parent[u], u, weight);
        }
        for (auto const &neighbor : get_adjacent_list(u)) {
          parent[neighbor.first] = u;
          stack.emplace_back(neighbor);
        }
      }
    }

    [[nodiscard]] bool is_connected() const {
      // empty graph
      if (empty()) {
        return false;
      }
      size_t tree_edge_num = 0;
      depth_first_search(
          *get_vertex_indices().begin(),
          [&tree_edge_num](auto, auto, auto) { tree_edge_num++; });
      return tree_edge_num + 1 == get_vertex_number();
    }

    [[nodiscard]] bool is_tree(size_t root = SIZE_MAX) const {
      // empty graph
      if (empty()) {
        return false;
      }
      size_t tree_edge_num = 0;
      if (root == SIZE_MAX) {
        root = *get_vertex_indices().begin();
      }
      depth_first_search(
          root, [&tree_edge_num](auto, auto, auto) { tree_edge_num++; });
      return tree_edge_num + 1 == get_vertex_number() &&
             tree_edge_num == get_edge_number();
    }

  protected:
    void add_directed_edge(const edge_type &e) {
      auto const first_index = add_vertex(e.first);
      auto const second_index = add_vertex(e.second);
      auto &neighbors = weighted_adjacent_list[first_index];
#ifndef NDEBUG
      auto it = std::ranges::find_if(neighbors, [second_index](auto const &a) {
        return a.first == second_index;
      });
      if (it != neighbors.end()) {
        throw std::runtime_error("edge has existed");
      }
#endif
      neighbors.emplace_back(second_index, e.weight);
    }
    bool remove_directed_edge(const edge_type &e) {
      return remove_directed_edge({get_vertex(e.first), get_vertex(e.second)});
    }
    bool remove_directed_edge(const indexed_edge &e) {
      auto it = weighted_adjacent_list.find(e.first);
      if (it == weighted_adjacent_list.end()) {
        return false;
      }
      auto &vertices = it->second;
      return vertices.remove_if(
                 [&](auto const &a) { return a.first == e.second; }) != 0;
    }

    std::unordered_map<size_t, std::list<std::pair<size_t, weight_type>>>
        weighted_adjacent_list;
    vertices_type vertex_pool;

  private:
    static inline std::list<std::pair<size_t, weight_type>> empty_adjacent_list;
  };

  template <typename T>
  concept IsGraph = requires(T a) {
    { a.get_vertex_indices() };
    { a.is_directed };
    { a.foreach_edge_with_weight() };
  };
} // namespace cyy::algorithm

namespace std {
  template <cyy::algorithm::IsGraph G> struct formatter<G> {
    // // Parses format specifications
    constexpr auto parse(format_parse_context &ctx) -> decltype(ctx.begin()) {
      // Parse the presentation format and store it in the formatter:
      const auto *it = ctx.begin();
      const auto *end = ctx.end();
      // Check if reached the end of the range:
      if (it != end && *it != '}') {
        throw format_error("invalid format");
      }
      // Return an iterator past the end of the parsed range:
      return it;
    }

    template <class FormatContext>
    auto format(const auto &g, FormatContext &ctx) {
      for (auto const &v : g.get_vertex_indices()) {
        std::format_to(ctx.out(), "vertex {}\n", v);
      }
      for (auto const &[indexed_edge, w] : g.foreach_edge_with_weight()) {
        auto e = g.get_edge(indexed_edge);
        std::format_to(ctx.out(), "{} {} {} with weight {}\n", e.first,
                       G::is_directed ? "->" : "<->", e.second, w);
      }
      return ctx.out();
    }
  };
} // namespace std
