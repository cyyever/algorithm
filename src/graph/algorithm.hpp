
/*!
 * \file graph_algorithm.hpp
 *
 * \brief implements graph_algorithm
 */

#pragma once

#include <list>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <vector>
#include <cassert>

#include "graph.hpp"

namespace cyy::algorithm {
    // breadth first search in g from s 
  template <typename vertex_type> 
std::unordered_map<size_t,path_node_ptr>
    breadth_first_search(const graph<vertex_type> &g,size_t s) {
       std::unordered_map<size_t,path_node_ptr> paths;
      std::unordered_map<size_t,float> total_cost;
      total_cost[s]=0;
      std::list<size_t> queue{s};
      while(!queue.empty()) {
        auto u=queue.front();
        queue.pop_front();
        std::cout<<"u is "<<u;
        for(auto const &neighbor:g.get_adjacent_list(u)) {
          auto const &[v,weight]=neighbor;
        std::cout<<"v is "<<v;
          if(!paths.contains(v)) {
            assert(u==s);
            auto node=std::make_shared<path_node>(v);
            paths.emplace(v,node);
            total_cost[v]=weight;
            queue.push_back(v);
          } else{
            auto new_cost=total_cost[u]+weight;
            if(new_cost <total_cost[v]) {
              auto prev_path=paths[u];
              prev_path->next= std::make_shared<path_node>(v);
              prev_path->next->prev=prev_path;
              total_cost[v]=new_cost;
              queue.push_back(v);
            }
          }
        }
      }
      return paths;
}
}
