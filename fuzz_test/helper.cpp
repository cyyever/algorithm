/*!
 * \file helper.cpp
 *
 * \brief fuzzing helper functions
 * \author cyy
 * \date 2019-02-12
 */
#include "helper.hpp"

#include <sys/endian.h>

cyy::algorithm::graph<uint8_t> fuzzing_graph(const uint8_t *Data, size_t Size) {
  cyy::algorithm::graph<uint8_t> g;
  size_t i = 0;
  while (i + 2 < Size) {
    g.add_edge({Data[i], Data[i + 1]});
    i += 2;
  }
  return g;
}
