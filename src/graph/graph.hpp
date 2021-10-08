/*!
 * \file graph.hpp
 *
 * \brief implements graph
 */

#pragma once

#include <algorithm>
#include <list>
#include <memory>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>

#include <boost/bimap.hpp>

namespace cyy::algorithm {
template <typename vertex_type> struct edge {
  vertex_type first;
  vertex_type second;
  float weight = 1;
  edge reverse() const { return {second, first, weight}; }
};
class path_node;

using path_node_ptr = std::shared_ptr<path_node>;
struct path_node {
  size_t vertex_index{};
  path_node_ptr prev{};
  path_node_ptr next{};
};

template <typename vertex_type> class graph {
public:
  using edge_type = edge<vertex_type>;
  using vertex_index_map_type = std::unordered_map<vertex_type, size_t>;
  using adjacent_matrix_type = std::vector<std::vector<float>>;
  graph() = default;
  ~graph() = default;
  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit graph(U edges) {
    for (auto const &edge : edges) {
      add_edge(edge);
    }
  }
  auto get_next_vertex_index() const { return next_vertex_index; }

  void add_edge(const edge<vertex_type> &e) {
    if (!add_directed_edge(e)) {
      return;
    }
    edge_num++;
    add_directed_edge(e.reverse());
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
    remove_directed_edge(e.reverse());
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
    for (auto const &[from_index, adjacent_vertices] : weighted_adjacent_list) {
      for (auto const &[to_index, weight] : adjacent_vertices) {
        adjacent_matrix[from_index][to_index] = weight;
      }
    }
    return adjacent_matrix;
  }

  auto get_vertices() const {
    return weighted_adjacent_list | std::views::keys;
  }
  size_t get_vertex_number() const { return vertex_indices.size(); }
  size_t get_edge_number() const { return edge_num; }
  const vertex_type &get_vertex(size_t index) const {
    return vertex_indices.right.at(index);
  }
  size_t get_vertex_index(const vertex_type &vertex) const {
    return vertex_indices.left.at(vertex);
  }

protected:
  bool add_directed_edge(const edge<vertex_type> &e) {
    auto first_index = add_vertex(e.first);
    auto second_index = add_vertex(e.second);
    auto &neighbors = weighted_adjacent_list[first_index];
    auto it = std::ranges::find_if(neighbors, [second_index](auto const &a) {
      return a.first == second_index;
    });
    if (it != neighbors.end()) {
      return false;
    }
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
  size_t add_vertex(vertex_type vertex) {
    auto it = vertex_indices.left.find(vertex);
    if (it != vertex_indices.left.end()) {
      return it->second;
    }
    vertex_indices.insert({std::move(vertex), next_vertex_index});
    return next_vertex_index++;
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
template <typename vertex_type>
class directed_graph : public graph<vertex_type> {
public:
  using edge_type = graph<vertex_type>::edge_type;
  directed_graph() = default;
  ~directed_graph() = default;
  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit directed_graph(U edges) {
    for (auto const &edge : edges) {
      add_edge(edge);
    }
  }
  directed_graph(edge_type e) { add_edge(e); }

  void add_edge(const edge_type &edge) {
    if (this->add_directed_edge(edge)) {
      this->edge_num++;
    }
  }
  void remove_edge(const edge_type &edge) {
    if (this->remove_directed_edge(edge)) {
      this->edge_num--;
    }
  }
  directed_graph get_transpose() const {
    directed_graph transpose;
    for (auto &[from_vertex, to_vertices] : this->weighted_adjacent_list) {
      for (auto &to_vertex : to_vertices) {
        transpose.add_edge({this->get_vertex(to_vertex.first),
                            this->get_vertex(from_vertex), to_vertex.second});
      }
    }
    return transpose;
  }
  std::vector<size_t> get_indegrees() const {
    std::vector<size_t> indegrees(this->get_next_vertex_index(), 0);
    for (auto const &[from_index, adjacent_vertices] :
         this->weighted_adjacent_list) {
      for (auto const &[to_index, weight] : adjacent_vertices) {
        indegrees[to_index]++;
      }
    }
    return indegrees;
  }
};

} // namespace cyy::algorithm
