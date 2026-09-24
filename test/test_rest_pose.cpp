#include "rest_pose.h"

#include <assert.h>
#include <stdio.h>

int main() {
  uint16_t reference[6] = {4000, 100, 200, 300, 400, 500};
  uint16_t raw[6] = {5, 100, 200, 300, 700, 200};
  RestPose::State state{};

  assert(RestPose::circularDistance(5, 4000) == 101);
  assert(RestPose::inside(raw, reference, RestPose::kEnterToleranceCounts));
  raw[4] = 743; // A5 is outside the 30-degree entry band.
  assert(!RestPose::inside(raw, reference, RestPose::kEnterToleranceCounts));
  raw[4] = 700;
  raw[5] = 843; // A6 now participates with the same wide band.
  assert(!RestPose::inside(raw, reference, RestPose::kEnterToleranceCounts));
  raw[5] = 200;
  assert(!RestPose::update(state, raw, reference, true, 100));
  assert(!RestPose::update(state, raw, reference, true, 599));
  assert(RestPose::update(state, raw, reference, true, 600));
  assert(state.in_region);

  raw[1] = 124; // Outside the 2-degree entry band, inside 3-degree exit band.
  assert(!RestPose::update(state, raw, reference, true, 700));
  assert(state.in_region);
  raw[1] = 236;
  assert(!RestPose::update(state, raw, reference, true, 800));
  assert(!RestPose::update(state, raw, reference, true, 1299));
  assert(RestPose::update(state, raw, reference, true, 1300));
  assert(!state.in_region);

  raw[1] = 100;
  assert(!RestPose::update(state, raw, reference, true, 1400));
  raw[1] = 124; // Losing the position condition resets the time condition.
  assert(!RestPose::update(state, raw, reference, true, 1700));
  raw[1] = 100;
  assert(!RestPose::update(state, raw, reference, true, 1800));
  assert(RestPose::update(state, raw, reference, true, 2300));

  // Sensor validity is part of the exit condition and also needs 500 ms.
  assert(!RestPose::update(state, raw, reference, false, 2400));
  assert(!RestPose::update(state, raw, reference, false, 2899));
  assert(RestPose::update(state, raw, reference, false, 2900));
  assert(!state.in_region);

  puts("rest pose tests: OK");
  return 0;
}
