
#pragma once

#include <unordered_map>

namespace cyy::algorithm {
  template <typename T> class object_pool {
  public:
    using element_id_type = size_t;

    element_id_type get_data_id(const T &elem) { return pool[elem]; }
    element_id_type add_data(const T &e) {
      auto [it, emplaced] = pool.emplace(e, next_object_id);
      if (emplaced) {
        next_object_id++;
      }
      return it->second;
    }

  private:
    std::unordered_map<T, element_id_type> pool;
    element_id_type next_object_id{0};
  };

} // namespace cyy::algorithm
