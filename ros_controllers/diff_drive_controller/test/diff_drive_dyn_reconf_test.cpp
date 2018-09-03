///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Jeremie Deray

#include "test_common.h"

#include "diff_drive_controller/DiffDriveControllerConfig.h"

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>

// TEST CASES
TEST_F(DiffDriveControllerTest, testDynReconfServerAlive)
{
  // wait for ROS
  while(!isControllerAlive() && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  if (!ros::ok())
    FAIL() << "Something went wrong while executing test";

  // Expect server is alive
  EXPECT_TRUE(ros::service::exists("diffbot_controller/set_parameters", true));

  dynamic_reconfigure::ReconfigureRequest  srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;

  // Expect server is callable (get-fashion)
  EXPECT_TRUE(ros::service::call("diffbot_controller/set_parameters", srv_req, srv_resp));

  EXPECT_EQ(1, srv_resp.config.bools.size());

  if (!srv_resp.config.bools.empty())
  {
    EXPECT_EQ("enable_odom_tf", srv_resp.config.bools[0].name);
    // expect false since it is set to false in the .test
    EXPECT_EQ(false, srv_resp.config.bools[0].value);
  }

  EXPECT_EQ(4, srv_resp.config.doubles.size());

  if (srv_resp.config.doubles.size() >= 4)
  {
    EXPECT_EQ("left_wheel_radius_multiplier", srv_resp.config.doubles[0].name);
    EXPECT_EQ(1, srv_resp.config.doubles[0].value);

    EXPECT_EQ("right_wheel_radius_multiplier", srv_resp.config.doubles[1].name);
    EXPECT_EQ(1, srv_resp.config.doubles[1].value);

    EXPECT_EQ("wheel_separation_multiplier", srv_resp.config.doubles[2].name);
    EXPECT_EQ(1, srv_resp.config.doubles[2].value);

    EXPECT_EQ("publish_rate", srv_resp.config.doubles[3].name);
    EXPECT_EQ(50, srv_resp.config.doubles[3].value);
  }

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "left_wheel_radius_multiplier";
  double_param.value = 0.95;

  srv_req.config.doubles.push_back(double_param);

  double_param.name = "right_wheel_radius_multiplier";
  double_param.value = 0.95;

  srv_req.config.doubles.push_back(double_param);

  double_param.name = "wheel_separation_multiplier";
  double_param.value = 0.95;

  srv_req.config.doubles.push_back(double_param);

  double_param.name = "publish_rate";
  double_param.value = 150;

  srv_req.config.doubles.push_back(double_param);

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "enable_odom_tf";
  bool_param.value = false;

  srv_req.config.bools.push_back(bool_param);

  // Expect server is callable (set-fashion)
  EXPECT_TRUE(ros::service::call("diffbot_controller/set_parameters", srv_req, srv_resp));

  EXPECT_EQ(1, srv_resp.config.bools.size());

  if (!srv_resp.config.bools.empty())
  {
    EXPECT_EQ("enable_odom_tf", srv_resp.config.bools[0].name);
    EXPECT_EQ(false, srv_resp.config.bools[0].value);
  }

  EXPECT_EQ(4, srv_resp.config.doubles.size());

  if (srv_resp.config.doubles.size() >= 4)
  {
    EXPECT_EQ("left_wheel_radius_multiplier", srv_resp.config.doubles[0].name);
    EXPECT_EQ(0.95, srv_resp.config.doubles[0].value);

    EXPECT_EQ("right_wheel_radius_multiplier", srv_resp.config.doubles[1].name);
    EXPECT_EQ(0.95, srv_resp.config.doubles[1].value);

    EXPECT_EQ("wheel_separation_multiplier", srv_resp.config.doubles[2].name);
    EXPECT_EQ(0.95, srv_resp.config.doubles[2].value);

    EXPECT_EQ("publish_rate", srv_resp.config.doubles[3].name);
    EXPECT_EQ(150, srv_resp.config.doubles[3].value);
  }
}

TEST_F(DiffDriveControllerTest, testDynReconfEnableTf)
{
  // wait for ROS
  while(!isControllerAlive() && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  if (!ros::ok())
    FAIL() << "Something went wrong while executing test";

  // set up tf listener
  tf::TransformListener listener;
  ros::Duration(2.0).sleep();
  // check the odom frame doesn't exist
  EXPECT_FALSE(listener.frameExists("odom"));

  dynamic_reconfigure::ReconfigureRequest  srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "enable_odom_tf";
  bool_param.value = true;

  srv_req.config.bools.push_back(bool_param);

  EXPECT_TRUE(ros::service::call("diffbot_controller/set_parameters", srv_req, srv_resp));

  ros::Duration(2.0).sleep();
  // check the odom frame doesn't exist
  EXPECT_TRUE(listener.frameExists("odom"));
}

TEST_F(DiffDriveControllerTest, testDynReconfClient)
{
  // wait for ROS
  while(!isControllerAlive() && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  if (!ros::ok())
    FAIL() << "Something went wrong while executing test";

  typedef diff_drive_controller::DiffDriveControllerConfig Config;

  dynamic_reconfigure::Client<Config> client("/diffbot_controller");

  // Assert one can get default params through dyn_reconf srv
  Config default_config;
  EXPECT_TRUE(client.getDefaultConfiguration(default_config, ros::Duration(0.5)));

  EXPECT_EQ(1,    default_config.left_wheel_radius_multiplier);
  EXPECT_EQ(1,    default_config.right_wheel_radius_multiplier);
  EXPECT_EQ(1,    default_config.wheel_separation_multiplier);
  EXPECT_EQ(true, default_config.enable_odom_tf);
  EXPECT_EQ(50,   default_config.publish_rate);

  // Assert one can get current params through dyn_reconf srv
  Config current_config;
  EXPECT_TRUE(client.getCurrentConfiguration(current_config, ros::Duration(0.5)));

  // Assert one can set params through dyn_reconf srv

  Config new_config;

  new_config.left_wheel_radius_multiplier  = 1.0123456;
  new_config.right_wheel_radius_multiplier = 1.0123456;
  new_config.wheel_separation_multiplier   = 1.0123456;

  /// @todo check default value
  new_config.enable_odom_tf = false;
  new_config.publish_rate = 42;

  EXPECT_TRUE(client.setConfiguration(new_config));

  ros::Duration(0.1).sleep();

  EXPECT_TRUE(client.getCurrentConfiguration(current_config, ros::Duration(0.5)));

  EXPECT_EQ(new_config.left_wheel_radius_multiplier,
            current_config.left_wheel_radius_multiplier);

  EXPECT_EQ(new_config.right_wheel_radius_multiplier,
            current_config.right_wheel_radius_multiplier);

  EXPECT_EQ(new_config.wheel_separation_multiplier,
            current_config.wheel_separation_multiplier);

  EXPECT_EQ(new_config.enable_odom_tf,
            current_config.enable_odom_tf);

  EXPECT_EQ(new_config.publish_rate,
            current_config.publish_rate);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "diff_drive_dyn_reconf_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
