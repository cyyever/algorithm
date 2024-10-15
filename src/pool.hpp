
#pragma once

#include <ranges>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace cyy::algorithm {
  template <typename T, bool ContinousID = true, typename Enable = void>

  class object_pool {
  public:
    using element_id_type = size_t;

    element_id_type get_data_id(const T &elem) const { return pool.at(elem); }
    element_id_type add_data(const T &e) {
      auto [it, emplaced] = pool.emplace(e, its.size());
      if (emplaced) {
        its.push_back(it);
      }
      return it->second;
    }
    auto size() const { return pool.size(); }
    bool empty() const { return pool.empty(); }
    bool contains(const T &e) const { return pool.contains(e); }
    bool contains_data_id(element_id_type id) const { return id < its.size(); }
    const T &get_data(element_id_type id) const { return its[id]->first; }
    auto foreach_data() const { return std::views::all(pool); }

  private:
    std::unordered_map<T, element_id_type> pool;
    std::vector<typename decltype(pool)::iterator> its;
  };

  template <typename T, bool ContinousID>

  class object_pool<T, ContinousID,
                    std::enable_if_t<std::is_trivial_v<T> && !ContinousID>> {
  public:
    using element_id_type = std::remove_cv_t<T>;

    element_id_type get_data_id(const T &elem) const { return elem; }
    element_id_type add_data(const T &e) const { return e; }
    T get_data(element_id_type id) const { return id; }
  };
} // namespace cyy::algorithm
