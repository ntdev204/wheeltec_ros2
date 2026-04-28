




#ifndef NAV2_AMCL__PF__PF_KDTREE_HPP_
#define NAV2_AMCL__PF__PF_KDTREE_HPP_

#ifdef INCLUDE_RTKGUI
#include <rtk.h>
#endif



typedef struct pf_kdtree_node
{

  int leaf, depth;


  int pivot_dim;
  double pivot_value;


  int key[3];


  double value;


  int cluster;


  struct pf_kdtree_node * children[2];
} pf_kdtree_node_t;



typedef struct
{

  double size[3];


  pf_kdtree_node_t * root;


  int node_count, node_max_count;
  pf_kdtree_node_t * nodes;


  int leaf_count;
} pf_kdtree_t;



extern pf_kdtree_t * pf_kdtree_alloc(int max_size);


extern void pf_kdtree_free(pf_kdtree_t * self);


extern void pf_kdtree_clear(pf_kdtree_t * self);


extern void pf_kdtree_insert(pf_kdtree_t * self, pf_vector_t pose, double value);


extern void pf_kdtree_cluster(pf_kdtree_t * self);





extern int pf_kdtree_get_cluster(pf_kdtree_t * self, pf_vector_t pose);


#ifdef INCLUDE_RTKGUI


extern void pf_kdtree_draw(pf_kdtree_t * self, rtk_fig_t * fig);

#endif

#endif
