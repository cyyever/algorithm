/*!
 * \file tree.hpp
 *
 * \brief implements tree
 */

#pragma once

#include <array>
#include <unordered_set>

#include "dag.hpp"
#include "graph.hpp"

namespace cyy::algorithm {

  template <typename vertex_type, typename weight_type = double>
  class tree : public graph<vertex_type, weight_type> {
  public:
    using edge_type = edge<vertex_type, weight_type>;
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
    size_t get_root() const { return root.value(); }
    void set_root(vertex_type root_) { root = this->get_vertex_index(root_); }
    void set_root_by_index(size_t root_) { root = root_; }

  protected:
    std::optional<size_t> root;
  };

  template <typename vertex_type, typename weight_type = double>
  class directed_tree_base : public DAG<vertex_type, weight_type> {
  public:
    using edge_type = edge<vertex_type, weight_type>;

    template <std::ranges::input_range U>
    requires std::same_as<edge_type, std::ranges::range_value_t<U>>
    directed_tree_base(U edges, vertex_type root_)
        : directed_tree_base(directed_graph<vertex_type>(edges), root(root_)) {}

    directed_tree_base(directed_graph<vertex_type, weight_type> g,
                       vertex_type root_)
        : DAG<vertex_type, weight_type>(std::move(g)) {
      root = this->get_vertex_index(root_);
    }
    // get a path from root to u
    std::vector<size_t> get_path(size_t u) const { return get_path(root, u); }

    // get a path from u to v
    std::vector<size_t> get_path(size_t u, size_t v) const {
      std::vector<size_t> path{u};
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

    auto get_root() const { return root; }

  protected:
    size_t root;
  };

  template <typename vertex_type, typename weight_type = double>
  class directed_tree : public directed_tree_base<vertex_type, weight_type> {
  public:
    using directed_tree_base<vertex_type, weight_type>::directed_tree_base;
    directed_tree(tree<vertex_type, weight_type> T) {
      this->root = T.get_root();
      this->vertex_indices = T.vertex_indices;
      T.breadth_first_search(this->root, [this](auto u, auto v, auto weight) {
        this->add_edge({this->get_vertex(u), this->get_vertex(v), weight});
      });
    }
  };

  template <typename vertex_type, typename weight_type = double>
  class in_directed_tree : public directed_tree_base<vertex_type, weight_type> {
  public:
    using edge_type = tree<vertex_type>::edge_type;
    using directed_tree_base<vertex_type, weight_type>::directed_tree_base;

    in_directed_tree(tree<vertex_type, weight_type> T) {
      this->root = T.get_root();
      this->vertex_indices = T.vertex_indices;
      T.breadth_first_search(this->root, [this](auto u, auto v, auto weight) {
        this->add_edge({this->get_vertex(v), this->get_vertex(u), weight});
      });
    }

    auto get_transpose() const {
      return directed_tree<vertex_type, weight_type>(
          directed_graph<vertex_type, weight_type>::get_transpose(),
          this->get_vertex(this->root));
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
      std::array<size_t, 2> frontier{u, v};
      std::unordered_set<size_t> parents{u, v};
      std::optional<size_t> ancestor_opt;
      while (!ancestor_opt) {
        for (size_t i = 0; i < frontier.size(); i++) {
          auto parent_opt = parent(frontier[i]);
          if (!parent_opt) {
            // root
            continue;
          }
          auto has_insersion = parents.emplace(*parent_opt).second;
          if (!has_insersion) {
            ancestor_opt = parent_opt;
            break;
          }
          frontier[i] = *parent_opt;
        }
      }
      return *ancestor_opt;
    }
  };

} // namespace cyy::algorithm
