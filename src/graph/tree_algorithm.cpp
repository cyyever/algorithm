/*!
 * \file tree_algorithm.cpp
 *
 * \brief
 */

#include "tree_algorithm.hpp"

#include <algorithm>
#include <set>
namespace cyy::algorithm {
  tree<size_t> recover_tree(const std::vector<size_t> &prufer_code) {
    std::vector<weighted_edge<size_t, double>> edges;
    std::set<size_t> vertices;
    for (size_t v = 0; v < prufer_code.size() + 2; v++) {
      if (std::ranges::find(prufer_code, v) == prufer_code.end()) {
        vertices.insert(v);
      }
    }

    for (auto it = prufer_code.begin(); it != prufer_code.end(); ++it) {
      auto const &v = *it;
      auto it2 = vertices.begin();
      edges.emplace_back(*it2, v);
      vertices.erase(it2);
      if (std::find(it + 1, prufer_code.end(), v) == prufer_code.end()) {
        vertices.insert(v);
      }
    }
    assert(vertices.size() == 2);
    edges.emplace_back(*vertices.begin(), *vertices.rbegin());

    return tree<size_t>(std::move(edges));
  }
} // namespace cyy::algorithm
