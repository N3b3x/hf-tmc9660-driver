#include "../../common/dummy_bus.hpp"
#include "TMC9660.hpp"

extern "C" void app_main() {
  DummyBus bus;
  TMC9660 driver(bus);

  driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
  driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR);
  driver.focControl.setTargetVelocity(1000);
}
