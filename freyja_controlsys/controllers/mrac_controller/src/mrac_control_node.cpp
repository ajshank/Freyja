#include <cstdio>
#include "freyja_control/freyja_control.hpp"
#include "mrac_estimator/mrac_estimator.hpp"

class MracControl : public FreyjaControl
{
  MracEstimator estimator_;

public:
  MracControl(std::string);
  ~MracControl()  { estimator_.stop(); }
  inline void init() { std::cout << "MRAC controller init!" << std::endl; }

  void computeFeedback() override;

};

MracControl::MracControl(std::string nodename) : FreyjaControl(nodename)
{
  estimator_.init();
  // init subscribers is done in base already

  /* associate publishers for controller output */
  atti_cmd_pub_ = create_publisher <RPYTCommand> ( "rpyt_command", 1 );

  // controller is run periodically; emulate 'events' therein if required
  float ctrl_period = 1.0/30.0;
  controller_timer_ = rclcpp::create_timer( this, get_clock(),
                                            std::chrono::duration<float>(ctrl_period),
                                            std::bind( &MracControl::computeFeedback,
                                                       this )
                                           );
}

void MracControl::computeFeedback()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *(get_clock()), 2000, "MRAC Controller active!");
  estimator_.setCurPosVel( getPVYCurState().head<6>() );
  estimator_.setRefPosVel( getPVYRefState().head<6>() );

}

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  rclcpp::init(argc, argv);

  auto mrac_node = std::make_shared<MracControl>("mrac");
  rclcpp::spin(mrac_node);

  rclcpp::shutdown();
  return 0;
}
