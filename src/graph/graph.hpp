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
};
class path_node;

using path_node_ptr = std::shared_ptr<path_node>;
struct path_node {
  size_t vertex_index;
  path_node_ptr prev{};
  path_node_ptr next{};
};

template <typename vertex_type> class graph {
public:
  using edge_type = edge<vertex_type>;
  // std::pair<vertex_type, vertex_type>;
  using vertex_index_map_type = std::unordered_map<vertex_type, size_t>;
  using adjacent_matrix_type = std::vector<std::vector<float>>;
  graph() = default;
  virtual ~graph() = default;
  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit graph(U edges) {
    for (auto const &edge : edges) {
      add_edge(edge);
    }
  }
  auto get_next_vertex_index() const { return next_vertex_index; }

  virtual void add_edge(const edge_type &edge) {
    add_directed_edge(edge);
    auto reversed_edge = edge;
    std::swap(reversed_edge.first, reversed_edge.second);
    add_directed_edge(reversed_edge);
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

  virtual void remove_edge(const edge_type &edge) {
    remove_directed_edge(edge);
    auto reversed_edge = edge;
    std::swap(reversed_edge.first, reversed_edge.second);
    remove_directed_edge(reversed_edge);
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
  size_t get_vertex_number() const { return vertex_indices.size(); }
  const vertex_type &get_vertex(size_t index) const {
    return vertex_indices.right.at(index);
  }
  size_t get_vertex_index(const vertex_type &vertex) const {
    return vertex_indices.left.at(vertex);
  }

protected:
  void add_directed_edge(const edge_type &edge) {
    auto first_index = add_vertex(edge.first);
    auto second_index = add_vertex(edge.second);
    weighted_adjacent_list[first_index].emplace_back(second_index, edge.weight);
  }
  void remove_directed_edge(const edge_type &edge) {
    auto first_index = get_vertex_index(edge.first);
    auto second_index = get_vertex_index(edge.second);
    auto it = weighted_adjacent_list.find(first_index);
    if (it == weighted_adjacent_list.end()) {
      return;
    }
    auto &vertices = it->second;
    const auto [first, last] =
        std::ranges::remove_if(vertices, [second_index](auto const &a) {
          return a.first == second_index;
        });
    vertices.erase(first, last);
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
  using graph<vertex_type>::graph;
  using edge_type = graph<vertex_type>::edge_type;
  void add_edge(const edge_type &edge) override {
    this->add_directed_edge(edge);
  }
  void remove_edge(const edge_type &edge) override {
    this->remove_directed_edge(edge);
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

template <typename vertex_type> class DAG : public directed_graph<vertex_type> {
public:
  using directed_graph<vertex_type>::directed_graph;
  using edge_type = directed_graph<vertex_type>::edge_type;

  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit DAG(U edges) : directed_graph<vertex_type>(edges) {
    if (!get_topological_ordering()) {
      throw std::logic_error("not a DAG");
    }
  }

  explicit DAG(directed_graph<vertex_type> g)
      : directed_graph<vertex_type>(std::move(g)) {
    if (!get_topological_ordering()) {
      throw std::logic_error("not a DAG");
    }
  }

  std::optional<std::vector<size_t>> get_topological_ordering() const {
    // Time Complexity is O(m+n)
    std::vector<size_t> order;
    order.reserve(this->get_vertex_number());
    std::vector<size_t> to_delete_vertices;
    auto indegrees=this->get_indegrees();
    for (size_t i = 0; i < indegrees.size(); i++) {
      if (indegrees[i] == 0) {
        to_delete_vertices.push_back(i);
      }
    }
    while (!to_delete_vertices.empty()) {
      auto u = to_delete_vertices.back();
      order.push_back(u);
      to_delete_vertices.pop_back();

      for (auto const &[to_index, weight] : this->get_adjacent_list(u)) {
        indegrees[to_index]--;
        if (indegrees[to_index] == 0) {
          to_delete_vertices.push_back(to_index);
        }
      }
    }
    // Not a DAG
    if (order.size() != this->get_vertex_number()) {
      return {};
    }
    return {order};
  }
};

} // namespace cyy::algorithm
