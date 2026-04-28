




#ifndef NAV2_AMCL__PF__PF_VECTOR_HPP_
#define NAV2_AMCL__PF__PF_VECTOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>


typedef struct
{
  double v[3];
} pf_vector_t;



typedef struct
{
  double m[3][3];
} pf_matrix_t;



pf_vector_t pf_vector_zero();











pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b);


pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b);






pf_matrix_t pf_matrix_zero();













void pf_matrix_unitary(pf_matrix_t * r, pf_matrix_t * d, pf_matrix_t a);

#ifdef __cplusplus
}
#endif

#endif
