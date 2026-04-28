




#ifndef NAV2_AMCL__MAP__MAP_HPP_
#define NAV2_AMCL__MAP__MAP_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


struct _rtk_fig_t;



#define MAP_WIFI_MAX_LEVELS 8



typedef struct
{

  int occ_state;


  double occ_dist;



} map_cell_t;



typedef struct
{

  double origin_x, origin_y;


  double scale;


  int size_x, size_y;


  map_cell_t * cells;



  double max_occ_dist;
} map_t;






map_t * map_alloc(void);


void map_free(map_t * map);


void map_update_cspace(map_t * map, double max_occ_dist);






double map_calc_range(map_t * map, double ox, double oy, double oa, double max_range);






void map_draw_occ(map_t * map, struct _rtk_fig_t * fig);


void map_draw_cspace(map_t * map, struct _rtk_fig_t * fig);


void map_draw_wifi(map_t * map, struct _rtk_fig_t * fig, int index);






#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)


#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)


#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))


#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

#ifdef __cplusplus
}
#endif

#endif
