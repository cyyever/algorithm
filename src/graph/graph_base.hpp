/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include <algorithm>
#include <list>
#include <memory>
#include <ranges>
#include <unordered_map>
#include <vector>

#include <boost/bimap.hpp>

namespace cyy::algorithm {
  template <typename vertex_type> struct edge {
    vertex_type first;
    vertex_type second;
    float weight = 1;
    auto operator<=>(const auto &rhs) const { return weight <=> rhs.weight; }
    auto operator==(const auto &rhs) const {
      return first == rhs.first && second == rhs.second;
    }
    edge reverse() const { return {second, first, weight}; }
  };

  struct indexed_edge {
    size_t first;
    size_t second;
    auto operator==(const auto &rhs) const {
      return first == rhs.first && second == rhs.second;
    }
    auto operator<=>(const auto &rhs) const {
      return std::tuple(first, second) <=> std::tuple(rhs.first, rhs.second);
    }
    indexed_edge reverse() const { return {second, first}; }
  };
} // namespace cyy::algorithm

namespace std {
  template <> struct hash<cyy::algorithm::indexed_edge> {
    size_t operator()(const cyy::algorithm::indexed_edge &x) const noexcept {
      return ::std::hash<size_t>()(x.first) ^ ::std::hash<size_t>()(x.second);
    }
  };
} // namespace std

namespace cyy::algorithm {
  template <typename vertex_type, bool directed> class graph_base {
  public:
    using edge_type = edge<vertex_type>;
    using vertex_index_map_type = std::unordered_map<vertex_type, size_t>;
    using adjacent_matrix_type = std::vector<std::vector<float>>;
    graph_base() = default;
    template <std::ranges::input_range U>
    requires std::same_as<edge_type, std::ranges::range_value_t<U>>
    explicit graph_base(U edges) {
      for (auto const &edge : edges) {
        add_edge(edge);
      }
    }
    auto get_next_vertex_index() const { return next_vertex_index; }

    void foreach_edge(std::function<void(indexed_edge)> edge_callback) const {
      for (auto const &[from_index, adjacent_vertices] :
           weighted_adjacent_list) {
        for (auto const &[to_index, weight] : adjacent_vertices) {
          if constexpr (directed) {
            edge_callback({from_index, to_index});
          } else {
            if (from_index <= to_index) {
              edge_callback({from_index, to_index});
            }
          }
        }
      }
    }

    size_t add_vertex(vertex_type vertex) {
      auto it = vertex_indices.left.find(vertex);
      if (it != vertex_indices.left.end()) {
        return it->second;
      }
      vertex_indices.insert({std::move(vertex), next_vertex_index});
      return next_vertex_index++;
    }

    edge<vertex_type> get_edge(const indexed_edge &edge) const {
      for (auto const &to_vertice : get_adjacent_list(edge.first)) {
        auto first_vertex = get_vertex(edge.first);
        auto second_vertex = get_vertex(edge.second);
        return {first_vertex, second_vertex, to_vertice.second};
      }
      throw std::runtime_error("no edge");
    }

    indexed_edge get_edge(const edge<vertex_type> &edge) const {
      return {get_vertex_index(edge.first), get_vertex_index(edge.second)};
    }

    void add_edge(const edge<vertex_type> &e) {
      if (!add_directed_edge(e)) {
        return;
      }
      edge_num++;
      if constexpr (!directed) {

        add_directed_edge(e.reverse());
      }
    }
    void remove_vertex(size_t vertex_index) {
      weighted_adjacent_list.erase(vertex_index);
      for (auto &[_, to_vertices] : weighted_adjacent_list) {
        const auto [first, last] =
            std::ranges::remove_if(to_vertices, [vertex_index](auto const &a) {
              return a.first == vertex_index;
            });
        to_vertices.erase(first, last);
      }
    }
    void remove_edge(const edge_type &e) {
      if (!remove_directed_edge(e)) {
        return;
      }
      edge_num--;
      if constexpr (!directed) {
        remove_directed_edge(e.reverse());
      }
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
      adjacent_matrix.reserve(next_vertex_index);

      for (auto const &[vertex, _] : weighted_adjacent_list) {
        adjacent_matrix.emplace_back(next_vertex_index, 0);
      }
      for (auto const &[from_index, adjacent_vertices] :
           weighted_adjacent_list) {
        for (auto const &[to_index, weight] : adjacent_vertices) {
          adjacent_matrix[from_index][to_index] = weight;
        }
      }
      return adjacent_matrix;
    }


    auto get_vertices() const {
      return std::views::all(vertex_indices.right) |
             std::views::transform([](auto const &it) { return it.first; });
    }
    size_t get_vertex_number() const { return vertex_indices.size(); }
    size_t get_edge_number() const { return edge_num; }
    const vertex_type &get_vertex(size_t index) const {
      return vertex_indices.right.at(index);
    }
    size_t get_vertex_index(const vertex_type &vertex) const {
      return vertex_indices.left.at(vertex);
    }
    void change_all_weights( float new_weight){

      for (auto &[_,to_vertices] :weighted_adjacent_list) {
      for (auto &to_vertice :to_vertices) {

          to_vertice.second=new_weight;
      }
      }
    }

    void change_weight( const indexed_edge &edge,           float new_weight){
      for (auto &to_vertice :weighted_adjacent_list.at(edge.first)) {
        if(to_vertice.first==edge.second) {
          to_vertice.second=new_weight;
        }
      }

    }

    // breadth first search in g from s
    void breadth_first_search(
        size_t s, std::function<void(size_t, size_t, float)> edge_fun) const {
      std::vector<bool> discovered(get_next_vertex_index(), false);
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
            edge_fun(u, v, weight);
            continue;
          }
        }
      }
    }
    // depth first search in g from s

    void recursive_depth_first_search(
        size_t s, std::function<void(size_t, size_t)> after_edge_fun) const {

      std::vector<bool> explored(get_next_vertex_index(), false);
      auto search_fun = [&](auto &&self, size_t u) {
        if (explored[u]) {
          return;
        }
        explored[u] = true;
        for (auto const &neighbor : get_adjacent_list(u)) {
          self(self, neighbor.first);
          after_edge_fun(u, neighbor.first);
        }
      };
      search_fun(search_fun, s);
    }

    // depth first search in g from s
    void depth_first_search(
        size_t s, std::function<void(size_t, size_t, float)> edge_fun) const {
      std::vector<bool> explored(get_next_vertex_index(), false);
      std::vector<std::pair<size_t, float>> stack{{s, 0}};
      std::vector<size_t> parent(get_next_vertex_index(), 0);

      while (!stack.empty()) {
        auto [u, weight] = stack.back();
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

    bool is_connected() const {
      size_t tree_edge_num = 0;
      depth_first_search(
          get_vertices()[0],
          [&tree_edge_num](auto u, auto v, float weight) { tree_edge_num++; });
      return tree_edge_num + 1 == get_vertex_number();
    }

    bool is_tree() const {
      size_t tree_edge_num = 0;
      depth_first_search(
          *get_vertices().begin(),
          [&tree_edge_num](auto u, auto v, float weight) { tree_edge_num++; });
      return tree_edge_num + 1 == get_vertex_number() &&
             tree_edge_num == get_edge_number();
    }

  protected:
    bool add_directed_edge(const edge<vertex_type> &e) {
      auto first_index = add_vertex(e.first);
      auto second_index = add_vertex(e.second);
      auto &neighbors = weighted_adjacent_list[first_index];
#ifndef NDEBUG
      auto it = std::ranges::find_if(neighbors, [second_index](auto const &a) {
        return a.first == second_index;
      });
      if (it != neighbors.end()) {
        return false;
      }
#endif
      neighbors.emplace_back(second_index, e.weight);
      return true;
    }
    bool remove_directed_edge(const edge<vertex_type> &e) {
      auto first_index = get_vertex_index(e.first);
      auto second_index = get_vertex_index(e.second);
      auto it = weighted_adjacent_list.find(first_index);
      if (it == weighted_adjacent_list.end()) {
        return false;
      }
      auto &vertices = it->second;
      return vertices.remove_if([second_index](auto const &a) {
        return a.first == second_index;
      }) != 0;
    }

  protected:
    size_t edge_num = 0;
    std::unordered_map<size_t, std::list<std::pair<size_t, float>>>
        weighted_adjacent_list;
    boost::bimap<vertex_type, size_t> vertex_indices;

  private:
    size_t next_vertex_index = 0;
    static inline std::list<std::pair<size_t, float>> empty_adjacent_list;
  };

} // namespace cyy::algorithm
