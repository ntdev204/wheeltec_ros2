








#include "test_constants/test_constants.h"

#include <vector>

const unsigned int g_valid_image_width = 10;
const unsigned int g_valid_image_height = 10;





const char g_valid_image_content[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  100, 100, 100, 100, 0, 0, 100, 100, 100, 0,
  100, 100, 100, 100, 0, 0, 100, 100, 100, 0,
  100, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  100, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  100, 0, 0, 0, 0, 0, 100, 100, 0, 0,
  100, 0, 0, 0, 0, 0, 100, 100, 0, 0,
  100, 0, 0, 0, 0, 0, 100, 100, 0, 0,
  100, 0, 0, 0, 0, 0, 100, 100, 0, 0,
  100, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

const char * g_valid_map_name = "testmap";
const char * g_valid_png_file = "testmap.png";
const char * g_valid_bmp_file = "testmap.bmp";
const char * g_valid_pgm_file = "testmap.pgm";
const char * g_valid_yaml_file = "testmap.yaml";
const char * g_tmp_dir = "/tmp";

const double g_valid_image_res = 0.1;
const std::vector<double> g_valid_origin{2.0, 3.0, 1.0};
const double g_default_free_thresh = 0.196;
const double g_default_occupied_thresh = 0.65;
