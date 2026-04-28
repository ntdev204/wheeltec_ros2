






#include <gtest/gtest.h>
#include <experimental/filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include "yaml-cpp/yaml.h"
#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server/map_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "test_constants/test_constants.h"

#define TEST_DIR TEST_DIRECTORY

using namespace std;
using namespace nav2_map_server;
using std::experimental::filesystem::path;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class MapIOTester : public ::testing::Test
{
protected:



  void fillLoadParameters(
    const std::string & image_file_name,
    LoadParameters & load_parameters)
  {
    load_parameters.image_file_name = image_file_name;
    load_parameters.resolution = g_valid_image_res;
    load_parameters.origin = g_valid_origin;
    load_parameters.free_thresh = g_default_free_thresh;
    load_parameters.occupied_thresh = g_default_occupied_thresh;
    load_parameters.mode = MapMode::Trinary;
    load_parameters.negate = 0;
  }




  void fillSaveParameters(
    const std::string & map_file_name,
    const std::string & image_format,
    SaveParameters & save_parameters)
  {
    save_parameters.map_file_name = map_file_name;
    save_parameters.image_format = image_format;
    save_parameters.free_thresh = g_default_free_thresh;
    save_parameters.occupied_thresh = g_default_occupied_thresh;
    save_parameters.mode = MapMode::Trinary;
  }



  void verifyMapMsg(const nav_msgs::msg::OccupancyGrid & map_msg)
  {
    ASSERT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
    ASSERT_EQ(map_msg.info.width, g_valid_image_width);
    ASSERT_EQ(map_msg.info.height, g_valid_image_height);
    for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
      ASSERT_EQ(g_valid_image_content[i], map_msg.data[i]);
    }
  }
};






TEST_F(MapIOTester, loadSaveValidPGM)
{

  LoadParameters loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_pgm_file), loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  verifyMapMsg(map_msg);


  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "pgm", saveParameters);

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));


  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}






TEST_F(MapIOTester, loadSaveValidPNG)
{

  LoadParameters loadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_png_file), loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  verifyMapMsg(map_msg);


  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "png", saveParameters);

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));


  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}






TEST_F(MapIOTester, loadSaveValidBMP)
{

  auto test_bmp = path(TEST_DIR) / path(g_valid_bmp_file);

  LoadParameters loadParameters;
  fillLoadParameters(test_bmp, loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_NO_THROW(loadMapFromFile(loadParameters, map_msg));

  verifyMapMsg(map_msg);


  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "bmp", saveParameters);

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));


  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}



TEST_F(MapIOTester, loadSaveMapModes)
{

  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(TEST_DIR) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);



  SaveParameters saveParameters;
  fillSaveParameters(path(g_tmp_dir) / path(g_valid_map_name), "png", saveParameters);
  saveParameters.mode = MapMode::Scale;

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));


  status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);


  saveParameters.mode = MapMode::Raw;

  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));


  status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);

  verifyMapMsg(map_msg);
}



TEST_F(MapIOTester, loadInvalidFile)
{

  auto test_invalid = path(TEST_DIR) / path("foo");

  LoadParameters loadParameters;
  fillLoadParameters(test_invalid, loadParameters);

  nav_msgs::msg::OccupancyGrid map_msg;
  ASSERT_ANY_THROW(loadMapFromFile(loadParameters, map_msg));


  LOAD_MAP_STATUS status = loadMapFromYaml("", map_msg);
  ASSERT_EQ(status, MAP_DOES_NOT_EXIST);

  status = loadMapFromYaml(std::string(test_invalid) + ".yaml", map_msg);
  ASSERT_EQ(status, INVALID_MAP_METADATA);
}



TEST_F(MapIOTester, saveInvalidParameters)
{

  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(TEST_DIR) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);


  SaveParameters saveParameters;

  saveParameters.map_file_name = path(g_tmp_dir) / path(g_valid_map_name);
  saveParameters.image_format = "";
  saveParameters.free_thresh = 2.0;
  saveParameters.occupied_thresh = 2.0;
  saveParameters.mode = MapMode::Trinary;
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));

  saveParameters.free_thresh = -2.0;
  saveParameters.occupied_thresh = -2.0;
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));

  saveParameters.free_thresh = 0.7;
  saveParameters.occupied_thresh = 0.2;
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));

  saveParameters.free_thresh = 0.0;
  saveParameters.occupied_thresh = 0.0;
  ASSERT_TRUE(saveMapToFile(map_msg, saveParameters));

  saveParameters.map_file_name = path("/invalid_path") / path(g_valid_map_name);
  ASSERT_FALSE(saveMapToFile(map_msg, saveParameters));
}


TEST_F(MapIOTester, loadValidYAML)
{
  LoadParameters loadParameters;
  ASSERT_NO_THROW(loadParameters = loadMapYaml(path(TEST_DIR) / path(g_valid_yaml_file)));

  LoadParameters refLoadParameters;
  fillLoadParameters(path(TEST_DIR) / path(g_valid_png_file), refLoadParameters);
  ASSERT_EQ(loadParameters.image_file_name, refLoadParameters.image_file_name);
  ASSERT_FLOAT_EQ(loadParameters.resolution, refLoadParameters.resolution);
  ASSERT_EQ(loadParameters.origin, refLoadParameters.origin);
  ASSERT_FLOAT_EQ(loadParameters.free_thresh, refLoadParameters.free_thresh);
  ASSERT_FLOAT_EQ(loadParameters.occupied_thresh, refLoadParameters.occupied_thresh);
  ASSERT_EQ(loadParameters.mode, refLoadParameters.mode);
  ASSERT_EQ(loadParameters.negate, refLoadParameters.negate);
}


TEST_F(MapIOTester, loadInvalidYAML)
{
  LoadParameters loadParameters;
  ASSERT_ANY_THROW(loadParameters = loadMapYaml(path(TEST_DIR) / path("invalid_file.yaml")));
}
