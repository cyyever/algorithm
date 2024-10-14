
#pragma once

#include <type_traits>
#include <unordered_map>
#include <vector>

namespace cyy::algorithm {
  template <typename T, bool ContinousID = true, typename Enable = void>

  class object_pool {
  public:
    using element_id_type = size_t;

    element_id_type get_data_id(const T &elem) { return pool[elem]; }
    element_id_type add_data(const T &e) {
      auto [it, emplaced] = pool.emplace(e, its.size());
      if (emplaced) {
        its.push_back(it);
      }
      return it->second;
    }
    const T &get_data(element_id_type id) const { return its[id]->first; }

  private:
    std::unordered_map<T, element_id_type> pool;
    std::vector<typename decltype(pool)::iterator> its;
  };

  template <typename T, bool ContinousID>

  class object_pool<T, ContinousID,
                    std::enable_if_t<std::is_trivial_v<T> && !ContinousID>> {
  public:
    using element_id_type = std::remove_cv_t<T>;

    element_id_type get_data_id(const T &elem) { return elem; }
    element_id_type add_data(const T &e) { return e; }
    const T &get_data(const element_id_type &id) const { return id; }
  };
} // namespace cyy::algorithm
