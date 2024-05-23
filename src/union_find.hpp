/*!
 * \file union_find.hpp
 *
 */

#pragma once
#include <memory>
#include <ranges>
#include <unordered_map>
namespace cyy::algorithm {
  template <typename data_type> class union_find {
  private:
    struct node;

  public:
    template <std::ranges::input_range U>
      requires std::same_as<data_type, std::ranges::range_value_t<U>>
    explicit union_find(U data) {
      if (data.size() == 0) {
        throw std::logic_error("data is empty");
      }
      nodes.reserve(data.size());
      for (auto const &item : data) {
        nodes.emplace(item, std::make_unique<node>());
      }
    }
    node *find(const data_type &item) {
      auto it = nodes.find(item);
      if (it == nodes.end()) {
        return nullptr;
      }
      return find(it->second.get());
    }

    void UNION(const data_type &a, const data_type &b) {
      UNION(find(a), find(b));
    }

    void UNION(node *a, node *b) {
      if (a == b) {
        return;
      }
      if (a->rank < b->rank) {
        a->representative = b;
      } else if (a->rank > b->rank) {
        b->representative = a;
      } else {
        a->representative = b;
        b->rank++;
      }
    }

  private:
    node *find(node *node_ptr) const {
      if (!node_ptr->representative) {
        return node_ptr;
      }
      node_ptr->representative = find(node_ptr->representative);
      return node_ptr->representative;
    }

    struct node {
      size_t rank{};
      node *representative = {};
    };
    std::unordered_map<data_type, std::unique_ptr<node>> nodes;
  };
} // namespace cyy::algorithm
