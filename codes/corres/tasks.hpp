/* significantly modified by Favonia for 15210-f15 */

/* COPYRIGHT (c) 2014 Umut Acar, Arthur Chargueraud, and Michael
 * Rainey
 * All rights reserved.
 *
 * \file tasks.hpp
 * \brief File that students are to use to enter solutions to the
 *  exercises that are assigned in the book.
 *
 */

#include <limits.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "sparray.hpp"
#include "graph.hpp"
#include "sort.hpp"

#ifndef _TASKS_H_
#define _TASKS_H_

/***********************************************************************/

namespace tasks {

/* 01010010 01100101 01100100 01110101 01100011 01100101 */

controller_type reduce_contr("reduce");

template <class Assoc_binop>
value_type reduce_rec(const Assoc_binop& f, const sparray& xs,
                      long lo, long hi) { 
  long ans;
  par::cstmt(reduce_contr, [&] {return hi - lo;},
    [&] {
      if (hi - lo < 2) ans = xs[lo];
      else {
        value_type left, right;
        long mid = (lo + hi) / 2;
        par::fork2([&] {left  = reduce_rec(f, xs, lo, mid);}, 
                   [&] {right = reduce_rec(f, xs, mid, hi);});
        ans = f(left, right);
      }
    }
  );
  return ans;
}

template <class Assoc_binop>
value_type reduce(const Assoc_binop& f, value_type b, const sparray& xs) {
  if (xs.size() == 0) return b;
  return reduce_rec(f, xs, 0, xs.size());
}

/*   __  __                    __ _                                    _
 *  |  \/  |   ___      _ _   / _` |   ___     ___     ___      _ _   | |_
 *  | |\/| |  / -_)    | '_|  \__, |  / -_)   (_-<    / _ \    | '_|  |  _|
 *  |_|__|_|  \___|   _|_|_   |___/   \___|   /__/_   \___/   _|_|_   _\__|
 *  _|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|
 *  "`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'
 */

controller_type mergesort_contr("mergesort");
void mergesort_rec(sparray& result, sparray& temp, long lo, long hi);
void merge(const sparray& xs, sparray& result, long result_lo,
           long Alo, long Ahi, long Blo, long Bhi);

/*@brief: Wrapper for the recursive mergesort*/
sparray mergesort(const sparray& input) {
  long len = input.size();
  // Result start off as input and gets more sorted
  sparray result = copy(input);
  // Allocated a temp upfront, so we don't allocate anything later.
  sparray temp = copy(input);
  mergesort_rec(result, temp, 0L, len);
  return result;
}

/*@brief: Mergesorts result in place, and uses temp
 *        to hold temporary changes.
 *@note: in_place_sort() is defined in sort.hpp
 *@param: lo is inclusive!
 *@param: hi is exclusive!
 */
void mergesort_rec(sparray& result, sparray& temp,
                   long lo, long hi) {
    long len = hi - lo;
    if(len < 2) return;
    par::cstmt(mergesort_contr, [&] { return nlogn(len); }, [&] {
      long mid = lo + len / 2;
      par::fork2([&] {
        mergesort_rec(result, temp, lo, mid);
      }, [&] {
        mergesort_rec(result, temp, mid, hi);
      });
      merge(result, temp, lo, lo, mid, mid, hi);
      // Copy from temp back into result
      prim::pcopy(&temp[0], &result[0], lo, hi, lo);
    }, [&] {
      in_place_sort(result, lo, hi); // Leave this alone!
    });
}

controller_type merge_contr("merge");
void merge(const sparray& xs, sparray& result, long result_lo,
           long Alo, long Ahi, long Blo, long Bhi) {
  long nA = Ahi - Alo; long nB = Bhi - Blo;
  par::cstmt(merge_contr, [&] {return nA;}, 
    [&] {
    if (nA < nB) merge(xs, result, result_lo, Blo, Bhi, Alo, Ahi);
    else if (nB < 1) prim::pcopy(&xs[0], &result[0], Alo, Ahi, result_lo);
    else if (nA == 1 && nB == 1) {
      long smaller = (xs[Alo] < xs[Blo] ? xs[Alo] : xs[Blo]);
      long bigger = (smaller == xs[Alo] ? xs[Blo] : xs[Alo]);
      par::fork2([&] {result[result_lo]   = smaller;},
                 [&] {result[result_lo+1] = bigger;});
    }
    else {
      long L1lo, L1hi, R1lo, R1hi;
      L1lo = Alo; L1hi = (Alo + Ahi)/2; R1lo = L1hi; R1hi = Ahi;
      long x = xs[R1lo]; long i = Blo;
      while (i < Bhi && xs[i] < x) i++;
      par::fork2(
        [&] {
          long L2lo = Blo; long L2hi = i;
          merge(xs, result, result_lo, L1lo, L1hi, L2lo, L2hi);
        },
        [&] {
          long R2lo = i; long R2hi = Bhi; 
          long L1len = L1hi - L1lo; long L2len = i - Blo;
          merge(xs, result, result_lo + L1len + L2len, R1lo, R1hi, R2lo, R2hi);
        });
    }
  }
  );
}

/*
 *  o--o  o--o  o-o
 *  |   | |    |
 *  O--o  O-o   o-o
 *  |   | |        |
 *  o--o  o    o--o
 *
 */

const vtxid_type not_a_vertexid = -1L;

loop_controller_type process_out_edges_contr("process_out_edges");
loop_controller_type edge_map_contr("edge_map");

sparray edge_map(const adjlist& graph,
                 std::atomic<bool>* visited,
                 const sparray& in_frontier) {
  long maxsize = graph.get_nb_vertices();
  long num_in_curr_front = in_frontier.size();
  std::atomic<bool>* out_frontier = my_malloc<std::atomic<bool>>(maxsize);
  std::atomic<long>* next_frontier = my_malloc<std::atomic<long>>(maxsize);
  par::parallel_for(edge_map_contr, 0L, maxsize, [&] (long i) {
    out_frontier[i].store(false);
    next_frontier[i].store(not_a_vertexid);
  });
  par::parallel_for(edge_map_contr, 0L, num_in_curr_front, [&] (long i) {
    vtxid_type vertex = in_frontier[i];
    visited[vertex].store(true);
    vtxid_type degree = graph.get_out_degree_of(vertex);
    neighbor_list out_edges = graph.get_out_edges_of(vertex);
    par::parallel_for(process_out_edges_contr, 0L, degree, [&] (long edge){
      vtxid_type other = out_edges[edge];
      bool is_visited = visited[other].load();
      if (!is_visited) {
        visited[other].store(true);
        bool f = false;
        bool success = out_frontier[other].compare_exchange_strong(f, true);
        if (success) next_frontier[other].store(other);
      }
    });
  });
  sparray xs = tabulate([&] (vtxid_type i) { return next_frontier[i].load(); }, 
    maxsize);
  sparray ans = filter([&] (vtxid_type i) {return (i != not_a_vertexid);} , xs);
  return ans;
}

loop_controller_type bfs_par_init_contr("bfs_init");
sparray bfs_par(const adjlist& graph, vtxid_type source) {
  long n = graph.get_nb_vertices();
  std::atomic<bool>* visited = my_malloc<std::atomic<bool>>(n);
  par::parallel_for(bfs_par_init_contr, 0L, n, [&] (long i) {
    visited[i].store(false);
  });
  visited[source].store(true);
  sparray cur_frontier = { source };
  while (cur_frontier.size() > 0)
    cur_frontier = edge_map(graph, visited, cur_frontier);
  sparray result = tabulate([&] (value_type i) {return visited[i].load();}, n);
  free(visited);
  return result;
}

sparray bfs(const adjlist& graph, vtxid_type source) {
#ifdef SEQUENTIAL_BASELINE
  return bfs_seq(graph, source);
#else
  return bfs_par(graph, source);
#endif
}

#endif /*! _TASKS_H_ */

} // end namespace

/***********************************************************************/
