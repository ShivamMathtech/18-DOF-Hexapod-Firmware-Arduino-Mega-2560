#include <cassert>
#include <cmath>

#include "Config.h"
#include "ConfigStore.h"
#include "GaitEngine.h"
#include "HexapodController.h"
#include "Kinematics.h"
#include "ServoConfig.h"

void setup();
void loop();

int main() {
  JointAngles angles{};
  const Vec3 nominal{config::DEFAULT_REACH_MM, 0.0f, -config::DEFAULT_BODY_HEIGHT_MM};
  assert(Kinematics::solveLeg(nominal, angles));
  assert(std::isfinite(angles.coxa));
  assert(std::isfinite(angles.femur));
  assert(std::isfinite(angles.tibia));

  const Vec3 unreachable{400.0f, 0.0f, -10.0f};
  assert(!Kinematics::solveLeg(unreachable, angles));

  for (uint8_t index = 0; index < config::SERVO_COUNT; ++index) {
    assert(SERVO_CALIBRATIONS[index].driver == (index < 9 ? 0 : 1));
    assert(SERVO_CALIBRATIONS[index].channel == index % 9);
  }

  ConfigStore store;
  float trims[config::SERVO_COUNT]{};
  assert(store.save(trims));
  float loaded[config::SERVO_COUNT]{};
  assert(store.load(loaded));

  HexapodController controller;
  assert(controller.begin());
  GaitEngine gait;
  gait.begin(controller);
  assert(gait.stand());
  assert(gait.setVelocity(0.25f, 0.0f, 0.0f));
  delay(config::CONTROL_INTERVAL_MS);
  assert(gait.update(millis()));
  assert(gait.stop());

  setup();
  loop();
  return 0;
}
