/*!
 * \file tree.hpp
 *
 * \brief implements tree
 */

#pragma once

#include "algorithm.hpp"
#include "graph.hpp"

namespace cyy::algorithm {

template <typename vertex_type> class tree : public graph<vertex_type> {
public:
  using graph<vertex_type>::graph;
  using edge_type = edge<vertex_type>;
  tree() = default;

  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit tree(U edges) : graph<vertex_type>(edges) {
    if (!is_tree(*this)) {
      throw std::logic_error("not a tree");
    }
  }
  void set_root(vertex_type root_) { root = this->get_vertex_index(root_); }

protected:
  std::optional<size_t> root;
};
template <typename vertex_type> class directed_tree : public tree<vertex_type> {
public:
  using tree<vertex_type>::tree;
  using edge_type = tree<vertex_type>::edge_type;
  directed_tree() = default;

  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit directed_tree(U edges, vertex_type root_) : tree<vertex_type>(edges) {
    set_root(root_);
  }
};
} // namespace cyy::algorithm
