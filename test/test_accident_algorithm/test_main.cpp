// Host (native) unit tests for the pure accident-detection algorithm.
// Run with:  pio test -e native
//
// These exercise AccidentAlgorithm in isolation — no MPU6050, no Wire, no
// Arduino — which is the whole point of extracting it from the hardware layer.

#include <unity.h>

#include "application/AccidentAlgorithm.h"

namespace {
constexpr int SENS = 2000;    // default AccidentConfig.sensitivity
constexpr int ANGLE = 60;     // default AccidentConfig.rollover_angle
constexpr int16_t GRAVITY_Z = 16384; // ~1g on Z at rest (MPU6050 +/-2g)
} // namespace

void setUp(void) {}
void tearDown(void) {}

// The vibration-damping counter starts at 2, so the very first reading after
// power-up must be swallowed (evaluated == false).
void test_first_sample_is_swallowed_by_vibration(void) {
  AccidentAlgorithm algo;
  AccidentSample s = algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 0.0f);
  TEST_ASSERT_FALSE(s.evaluated);
}

// A sudden high-G delta at low speed is a validated impact.
void test_impact_detected_low_speed(void) {
  AccidentAlgorithm algo;
  algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 0.0f); // vibration skip
  algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 0.0f); // baseline
  AccidentSample s = algo.evaluate(3000, 0, GRAVITY_Z, SENS, ANGLE, 0.0f);

  TEST_ASSERT_TRUE(s.evaluated);
  TEST_ASSERT_TRUE(s.impact_detected);
  TEST_ASSERT_FALSE(s.rollover_detected);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 3000.0f, s.impact_magnitude);
}

// The same delta at sustained high speed is rejected (rough-road false pos).
void test_impact_rejected_at_high_speed(void) {
  AccidentAlgorithm algo;
  algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 60.0f);
  algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 60.0f);
  AccidentSample s = algo.evaluate(3000, 0, GRAVITY_Z, SENS, ANGLE, 60.0f);

  TEST_ASSERT_TRUE(s.evaluated);
  TEST_ASSERT_FALSE(s.impact_detected);
}

// A delta below the sensitivity threshold is not an impact.
void test_below_threshold_no_impact(void) {
  AccidentAlgorithm algo;
  algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 0.0f);
  algo.evaluate(0, 0, GRAVITY_Z, SENS, ANGLE, 0.0f);
  AccidentSample s = algo.evaluate(500, 0, GRAVITY_Z, SENS, ANGLE, 0.0f);

  TEST_ASSERT_TRUE(s.evaluated);
  TEST_ASSERT_FALSE(s.impact_detected);
}

// A steady 90-degree tilt (gravity on Y) is a rollover, not an impact.
void test_rollover_detected(void) {
  AccidentAlgorithm algo;
  algo.evaluate(0, GRAVITY_Z, 0, SENS, ANGLE, 0.0f); // vibration skip + baseline
  AccidentSample s = algo.evaluate(0, GRAVITY_Z, 0, SENS, ANGLE, 0.0f);

  TEST_ASSERT_TRUE(s.evaluated);
  TEST_ASSERT_TRUE(s.rollover_detected);
  TEST_ASSERT_FALSE(s.impact_detected);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_sample_is_swallowed_by_vibration);
  RUN_TEST(test_impact_detected_low_speed);
  RUN_TEST(test_impact_rejected_at_high_speed);
  RUN_TEST(test_below_threshold_no_impact);
  RUN_TEST(test_rollover_detected);
  return UNITY_END();
}
