#include <gtest/gtest.h>
#include <ros/ros.h>

int my_argc;
char** my_argv;

TEST(URDF, CorrectFormat)
{
  int test_result = 0;

  // check if input file is given
  if ( my_argc <= 1 )
  {
    ROS_ERROR("No urdf file given");
    EXPECT_TRUE( false );
  }
  
  // urdf.xacro to test
  std::string file_to_test = std::string(my_argv[1]);
  ROS_INFO("testing: %s", file_to_test.c_str());
  
  // convert xacro to urdf
  std::string convert = "python `rospack find xacro`/xacro.py " + file_to_test + " > tmp.urdf";
  test_result = system(convert.c_str());
  EXPECT_TRUE(test_result == 0);
  
  // do urdf check
  test_result = system("`rospack find urdf_parser`/bin/check_urdf tmp.urdf");
  EXPECT_TRUE(test_result == 0);
}

int main(int argc, char **argv)
{
    my_argc = argc;
    my_argv = argv;
	
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
