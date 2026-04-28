




#ifndef NAV2_AMCL__PF__PF_PDF_HPP_
#define NAV2_AMCL__PF__PF_PDF_HPP_

#include "nav2_amcl/pf/pf_vector.hpp"




#ifdef __cplusplus
extern "C" {
#endif





typedef struct
{

  pf_vector_t x;
  pf_matrix_t cx;

  double cxdet;


  pf_matrix_t cr;
  pf_vector_t cd;



} pf_pdf_gaussian_t;



pf_pdf_gaussian_t * pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx);


void pf_pdf_gaussian_free(pf_pdf_gaussian_t * pdf);








double pf_ran_gaussian(double sigma);


pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t * pdf);

#ifdef __cplusplus
}
#endif

#endif
