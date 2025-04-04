/*!
 * \file union_find.hpp
 *
 */

#pragma once
#include <cassert>
import std;
namespace cyy::algorithm {
  template <typename data_type> class union_find {

  public:
    struct node {
      data_type data;
      std::size_t rank{};
      std::shared_ptr<node> representative;
    };
    using node_ptr = std::shared_ptr<node>;

    static node_ptr make_set(const data_type &item) {
      return std::make_shared<node>(item);
    }
    static const node *find(const node_ptr &ptr) {
      if (!ptr) {
        return nullptr;
      }
      return find_impl(ptr).get();
    }

    static void UNION(const node_ptr &a, const node_ptr &b) {
      if (!a || !b || a == b) {
        return;
      }
      const auto &a_root = find_impl(a);
      const auto &b_root = find_impl(b);
      if (a_root->rank < b_root->rank) {
        a_root->representative = b;
      } else if (a_root->rank > b_root->rank) {
        b_root->representative = a;
      } else {
        a_root->representative = b;
        b_root->rank++;
      }
      assert(find(a) == find(b));
    }

  private:
    static const node_ptr &find_impl(const node_ptr &ptr) {
      if (!ptr->representative) {
        return ptr;
      }
      ptr->representative = find_impl(ptr->representative);
      return ptr->representative;
    }
  };
} // namespace cyy::algorithm
