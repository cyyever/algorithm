/*!
 * \file tree.hpp
 *
 * \brief implements tree
 */

#pragma once

#include <list>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>

#include "algorithm.hpp"
#include "graph.hpp"

namespace cyy::algorithm {

template <typename vertex_type = size_t>
class tree : public graph<vertex_type> {
public:
  using graph<vertex_type>::graph;
  using edge_type = graph<vertex_type>::edge_type;
  tree() = default;

  template <std::ranges::input_range U>
  requires std::same_as<edge_type, std::ranges::range_value_t<U>>
  explicit tree(U edges) : graph<vertex_type>(edges) {
    if (!is_tree(*this)) {
      throw std::logic_error("not a tree");
    }
  }
  void set_root(vertex_type root_) { root = root_; }

private:
  std::optional<vertex_type> root;
};
} // namespace cyy::algorithm
