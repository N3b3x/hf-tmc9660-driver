#include "TMC9660.hpp"
#include "../../common/dummy_bus.hpp"

extern "C" void app_main()
{
    DummyBus bus;
    TMC9660 driver(bus);

    driver.configureMotorType(TMC9660::MotorType::BLDC, 7);
    driver.setCommutationMode(TMC9660::CommutationMode::FOC_HALL);
    driver.setTargetVelocity(1000);
}
