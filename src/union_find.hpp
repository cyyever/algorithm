/*!
 * \file union_find.hpp
 *
 */

#pragma once
#include <cassert>
#include <ranges>
#include <unordered_map>
#include <memory>
namespace cyy::algorithm {
template <typename data_type> 
  class union_find {
private:
  struct node;
public:
  template <std::ranges::input_range U>
  requires std::same_as<data_type, std::ranges::range_value_t<U>>
  explicit union_find(U data) {
    if(data.size()==0) {
      throw std::logic_error("data is empty");
    }
    nodes.reserve(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      auto p = std::make_shared<node>();
      nodes.emplace(data[i], p);
    }
  }
  std::shared_ptr<node> find(const data_type &item) {
    auto representative = find(nodes[item]);
    return representative;
  }

  void UNION(const data_type &a, const data_type &b) {
    UNION(find(a), find(b));
  }

private:
  void UNION(std::shared_ptr<node> a, std::shared_ptr<node> b) {
    if (a == b) {
      return;
    }
    if (a->rank < b->rank) {
      a->representative = b;
    } else if (a->rank > b->rank) {
      b->representative = a;
    } else {
      a->representative = b;
      a->rank++;
    }
  }

  std::shared_ptr<node> find(std::shared_ptr<node> &node_ptr) {
    if (!node_ptr->representative) {
      return node_ptr;
    }
    node_ptr->representative = find(node_ptr->representative);
    return node_ptr->representative;
  }

  struct node {
    size_t rank{};
    std::shared_ptr<node> representative = {};
  };
  std::unordered_map<data_type, std::shared_ptr<node>> nodes;
};
} // namespace cyy::algorithm
