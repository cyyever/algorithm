/*!
 * \file tree.hpp
 *
 * \brief implements tree
 */

#pragma once

#include <unordered_set>

#include "graph.hpp"

namespace cyy::algorithm {

  template <typename vertex_type> class tree : public graph<vertex_type> {
  public:
    using edge_type = edge<vertex_type>;
    tree() = default;

    template <std::ranges::input_range U>
    requires std::same_as<edge_type, std::ranges::range_value_t<U>>
    explicit tree(U edges, bool check = true) : graph<vertex_type>(edges) {
      if (check && !this->is_tree()) {
        throw std::logic_error("not a tree");
      }
    }
    explicit tree(graph<vertex_type> g, bool check = true)
        : graph<vertex_type>(std::move(g)) {
      if (check && !this->is_tree()) {
        throw std::logic_error("not a tree");
      }
    }

    void set_root(vertex_type root_) { root = this->get_vertex_index(root_); }
    void set_root_by_index(size_t root_) { root = root_; }

  protected:
    std::optional<size_t> root;
  };

  template <typename vertex_type>
  class directed_tree_base : public directed_graph<vertex_type> {
  public:
    using edge_type = edge<vertex_type>;

    template <std::ranges::input_range U>
    requires std::same_as<edge_type, std::ranges::range_value_t<U>>
    directed_tree_base(U edges, vertex_type root_, bool check)
        : directed_tree_base(directed_graph<vertex_type>(edges), root(root_)) {}

    directed_tree_base(directed_graph<vertex_type> g, vertex_type root_,
                       bool check = false)
        : directed_graph<vertex_type>(std::move(g)) {
      root = this->get_vertex_index(root_);
      auto rootaaa = this->get_vertex(root);
      /* if (check && !this->is_tree(root)) { */
      /*   throw std::logic_error("not a tree"); */
      /* } */
    }

    std::vector<size_t> get_path(size_t u) const {
      std::vector<size_t> path;
      while (u != root) {
        u = *(this->get_adjacent_list(u).begin());
        path.push_back(u);
      }
      return path;
    }
    auto get_root() const { return root; }

  protected:
    size_t root;
  };

  template <typename vertex_type>
  class directed_tree : public directed_tree_base<vertex_type> {
  public:
    using directed_tree_base<vertex_type>::directed_tree_base;
  };

  template <typename vertex_type>
  class in_directed_tree : public directed_tree_base<vertex_type> {
  public:
    using edge_type = tree<vertex_type>::edge_type;
    using directed_tree_base<vertex_type>::directed_tree_base;

    directed_tree<vertex_type> get_transpose() const {
      directed_tree<vertex_type> T(directed_graph<vertex_type>::get_transpose(),
                                   this->get_vertex(this->root), false);
      return T;
    }

    std::optional<size_t> parent(size_t u) const {
      auto const &l = this->get_adjacent_list(u);
      if (l.empty()) {
        return {};
      }
      return l.begin()->first;
    }

    std::vector<size_t> get_leaves() const {
      std::vector<size_t> leaves;
      auto indegrees = this->get_indegrees();
      for (size_t idx = 0; idx < indegrees.size(); idx++) {
        if (indegrees[idx] == 0 && this->vertex_indices.right.find(idx) !=
                                       this->vertex_indices.right.end()) {

          leaves.push_back(idx);
        }
      }
      return leaves;
    }

    size_t nearest_ancestor(size_t u, size_t v) const {
      if (u == v) {
        return v;
      }
      std::vector<size_t> frontier{u, v};
      std::unordered_set<size_t> parents{u, v};
      while (true) {
        for (size_t i = 0; i < frontier.size(); i++) {
          auto parent_opt = parent(frontier[i]);
          if (!parent_opt) {
            // root
            continue;
          }
          auto has_insersion = parents.emplace(*parent_opt).second;
          if (!has_insersion) {
            return *parent_opt;
          }
          frontier[i] = *parent_opt;
        }
      }
      throw std::runtime_error("shouldn't be here");
    }
  };
} // namespace cyy::algorithm
