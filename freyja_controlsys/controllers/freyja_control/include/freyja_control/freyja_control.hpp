#pragma once

#ifndef FREYJA_CONTROL_HPP
#define FREYJA_CONTROL_HPP

#include <mutex>
#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/ctrl_command.hpp"
#include <freyja_msgs/msg/controller_debug.hpp>
#include <freyja_msgs/msg/reference_state.hpp>
#include "freyja_msgs/msg/current_state_named.hpp"

#include <eigen3/Eigen/Dense>

typedef freyja_msgs::msg::ReferenceState      ReferenceState;
typedef freyja_msgs::msg::CurrentState        CurrentState;
typedef std_srvs::srv::SetBool                BoolServ;
typedef freyja_msgs::msg::CtrlCommand         RPYTCommand;
typedef freyja_msgs::msg::ControllerDebug     CTRLDebug;
typedef geometry_msgs::msg::Vector3Stamped    GeomVec3Stamped;
typedef freyja_msgs::msg::CurrentStateNamed   CurrentStateNamed;

typedef Eigen::Matrix<double, 1, 1>           Vector1d;
typedef Eigen::Matrix<double, 4, 1>           Vector4d;
typedef Eigen::Matrix<double, 6, 1>           PosVelNED;
typedef Eigen::Matrix<double, 9, 1>           PosVelAccNED;
typedef Eigen::Matrix<double, 7, 1>           PosVelYawNED;
typedef Eigen::Matrix<double, 4, 7>           BaseFdbMatrix;

using std::placeholders::_1;
using std::placeholders::_2;

struct FreyjaControlOptions
{
  bool  ctrl_enable_;
  bool  ctrl_use_ff_;
  float ctrl_rate_;
};

class FreyjaControl: public rclcpp::Node
{
protected:
  CurrentStateNamed curstate_;
  ReferenceState    refstate_;
  Vector4d          ref_accel_ff_;

  PosVelYawNED      pvy_curstate_;
  PosVelYawNED      pvy_refstate_;

  Vector4d          basectrl_lqr_u_acc_;    // world frame
  Vector4d          basectrl_body_rpyt_;
  BaseFdbMatrix     basectrl_lqr_k_;

  FreyjaControlOptions  basectrl_opts_;

private:
  rclcpp::TimerBase::SharedPtr basectrl_timer_;
  void preinitContainers();
  inline void updateLocalCurState()
  {
    pvy_curstate_.segment<3>(0) = Eigen::Map<const Eigen::Vector3d>(curstate_.pos_ned.data());
    pvy_curstate_.segment<3>(3) = Eigen::Map<const Eigen::Vector3d>(curstate_.vel_ned.data());
    pvy_curstate_.coeffRef(6) = curstate_.ang_rpy[2];
  }

  inline void updateLocalRefState()
  {
    pvy_refstate_.segment<3>(0) << refstate_.pn, refstate_.pe, refstate_.pd;
    pvy_refstate_.segment<3>(3) << refstate_.vn, refstate_.ve, refstate_.vd;
    pvy_refstate_.coeffRef(6) = refstate_.yaw;

    ref_accel_ff_ << refstate_.an, refstate_.ae, refstate_.ad, 0.0;
  }

public:
  FreyjaControl(std::string nodename) : Node(nodename)
  {
    preinitContainers();
    curstate_sub_ = create_subscription<CurrentStateNamed>( "current_state", 1,
                            [&](const CurrentStateNamed::ConstSharedPtr msg)
                            { curstate_ = *msg; updateLocalCurState(); } );
    refstate_sub_ = create_subscription<ReferenceState>( "reference_state", 1,
                            [&](const ReferenceState::ConstSharedPtr msg)
                            { refstate_ = *msg; updateLocalRefState(); } );
  }
  FreyjaControl(std::string nodename, FreyjaControlOptions opts) : FreyjaControl(nodename)
  {
    basectrl_opts_ = opts;
    if(basectrl_opts_.ctrl_enable_)
    {
      RCLCPP_INFO(get_logger(), "Background base-controller active!");
      initBaseController();
      float interv = 1.0/basectrl_opts_.ctrl_rate_;
      basectrl_timer_ = rclcpp::create_timer(this, get_clock(),
                                    std::chrono::duration<float>(interv),
                                    std::bind(&FreyjaControl::baseCtrlComputeFeedback,
                                              this));
    }
  }

  // force implementation to define at least one init fcn
  virtual void init() = 0;

  // state callback has an empty implementation for base class
  rclcpp::Subscription<CurrentStateNamed>::SharedPtr curstate_sub_;

  // reference callback has an empty implementation for base class
  rclcpp::Subscription<ReferenceState>::SharedPtr refstate_sub_;

  // provide timer object, but force implementation to define `computeFeedback`
  rclcpp::TimerBase::SharedPtr controller_timer_;
  virtual void computeFeedback() = 0;

  void initBaseController();
  void baseCtrlComputeFeedback();

  rclcpp::Publisher<RPYTCommand>::SharedPtr atti_cmd_pub_;
  rclcpp::Publisher<CTRLDebug>::SharedPtr controller_debug_pub_;

  PosVelYawNED getPVYCurState() { return pvy_curstate_; }
  PosVelYawNED getPVYRefState() { return pvy_refstate_; }
};

#endif
