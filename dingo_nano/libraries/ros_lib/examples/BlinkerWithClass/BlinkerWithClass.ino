#include <ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

class Blinker
{
public:
  Blinker(byte pin, uint16_t period)
  : pin_(pin), period_(period),
    subscriber_("set_blink_period", &Blinker::set_period_callback, this),
    service_server_("activate_blinker", &Blinker::service_callback, this)
  {}

  void init(ros::NodeHandle& nh)
  {
    pinMode(pin_, OUTPUT);
    nh.subscribe(subscriber_);
    nh.advertiseService(service_server_);
  }

  void run()
  {
    if(active_ && ((millis() - last_time_) >= period_))
    {
      last_time_ = millis();
      digitalWrite(pin_, !digitalRead(pin_));
    }
  }

  void set_period_callback(const std_msgs::UInt16& msg)
  {
    period_ = msg.data;
  }

  void service_callback(const std_srvs::SetBool::Request& req,
                              std_srvs::SetBool::Response& res)
  {
    active_ = req.data;
    res.success = true;
    if(req.data)
      res.message = "Blinker ON";
    else
      res.message = "Blinker OFF";
  }

private:
  const byte pin_;
  bool active_ = true;
  uint16_t period_;
  uint32_t last_time_;
  ros::Subscriber<std_msgs::UInt16, Blinker> subscriber_;
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, Blinker> service_server_;
};

Blinker blinker(LED_BUILTIN, 500);

void setup()
{
  nh.initNode();
  blinker.init(nh);
}

void loop()
{
  blinker.run();
  nh.spinOnce();
  delay(1);
}
