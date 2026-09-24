#include "wrist_orientation.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

namespace {
constexpr double kPi = 3.14159265358979323846;

double radians(int16_t counts) { return counts * 2.0 * kPi / 4096.0; }

void multiply(const double a[3][3], const double b[3][3], double out[3][3]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      out[r][c] = 0;
      for (int k = 0; k < 3; ++k) out[r][c] += a[r][k] * b[k][c];
    }
}

void rotation(char axis, double angle, double out[3][3]) {
  const double c = cos(angle), s = sin(angle);
  const double identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int r = 0; r < 3; ++r)
    for (int col = 0; col < 3; ++col) out[r][col] = identity[r][col];
  if (axis == 'X') {
    out[1][1] = c; out[1][2] = -s; out[2][1] = s; out[2][2] = c;
  } else if (axis == 'Y') {
    out[0][0] = c; out[0][2] = s; out[2][0] = -s; out[2][2] = c;
  } else {
    out[0][0] = c; out[0][1] = -s; out[1][0] = s; out[1][1] = c;
  }
}

void compose(const char order[4], const int16_t angles[3], double out[3][3]) {
  double value[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int i = 0; i < 3; ++i) {
    double step[3][3], next[3][3];
    rotation(order[i], radians(angles[i]), step);
    multiply(value, step, next);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) value[r][c] = next[r][c];
  }
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) out[r][c] = value[r][c];
}

double matrixError(const int16_t xzy[3], const int16_t zxz[3]) {
  double a[3][3], b[3][3];
  const int16_t webMapped[3] = {
      (int16_t)-xzy[0], xzy[1], (int16_t)-xzy[2]};
  compose("XZY", webMapped, a);
  compose("ZXZ", zxz, b);
  double error = 0;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      error = fmax(error, fabs(a[r][c] - b[r][c]));
  return error;
}

int16_t degreeCounts(int degrees) {
  return (int16_t)lround(degrees * 4096.0 / 360.0);
}

int circularDelta(int16_t current, int16_t previous) {
  int delta = current - previous;
  if (delta > 2047) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return delta;
}
} // namespace

int main() {
  WristOrientation::State state{};
  int16_t in[3] = {398, -284, 626};
  int16_t out[3];
  WristOrientation::xzyToZxz(in, out, state);
  assert(matrixError(in, out) < 0.002);

  WristOrientation::reset(state);
  int16_t previous[3] = {0, 0, 0};
  bool havePrevious = false;
  for (int x = -100; x <= 100; x += 4) {
    int16_t crossing[3] = {(int16_t)x, 341, 0};
    WristOrientation::xzyToZxz(crossing, out, state);
    if (havePrevious) {
      for (int i = 0; i < 3; ++i) {
        int delta = out[i] - previous[i];
        if (delta > 2047) delta -= 4096;
        if (delta < -2048) delta += 4096;
        assert(abs(delta) < 128);
      }
    }
    for (int i = 0; i < 3; ++i) previous[i] = out[i];
    havePrevious = true;
  }

  WristOrientation::reset(state);
  havePrevious = false;
  for (int x = 1900; x <= 2196; x += 4) {
    int wrapped = x > 2047 ? x - 4096 : x;
    int16_t crossing[3] = {(int16_t)wrapped, 0, 0};
    WristOrientation::xzyToZxz(crossing, out, state);
    if (havePrevious) {
      for (int i = 0; i < 3; ++i) {
        int delta = out[i] - previous[i];
        if (delta > 2047) delta -= 4096;
        if (delta < -2048) delta += 4096;
        assert(abs(delta) < 128);
      }
    }
    for (int i = 0; i < 3; ++i) previous[i] = out[i];
    havePrevious = true;
  }

  // Match the web tracker's 3-degree entry, 5-degree exit and 9-degree
  // per-output recovery. Raw A4 is reversed by the web/firmware mapping.
  WristOrientation::reset(state);
  int16_t tracking[3] = {(int16_t)-degreeCounts(10), 0, 0};
  WristOrientation::xzyToZxz(tracking, out, state);
  assert(!state.singular);
  tracking[0] = (int16_t)-degreeCounts(2);
  WristOrientation::xzyToZxz(tracking, out, state);
  assert(state.singular);
  tracking[0] = 0;
  tracking[1] = degreeCounts(100);
  WristOrientation::xzyToZxz(tracking, out, state);
  tracking[0] = (int16_t)-degreeCounts(4);
  WristOrientation::xzyToZxz(tracking, out, state);
  assert(state.singular);
  int16_t beforeExit[3] = {out[0], out[1], out[2]};
  tracking[0] = (int16_t)-degreeCounts(6);
  WristOrientation::xzyToZxz(tracking, out, state);
  assert(!state.singular && state.recovering);
  for (int i = 0; i < 3; ++i)
    assert(abs(circularDelta(out[i], beforeExit[i])) <= degreeCounts(9) + 1);
  for (int i = 0; i < 100 && state.recovering; ++i)
    WristOrientation::xzyToZxz(tracking, out, state);
  assert(!state.recovering);
  assert(matrixError(tracking, out) < 0.002);

  struct WebGoldenFrame {
    int16_t input[3];
    int16_t output[3];
    bool singular;
    bool recovering;
  };
  const WebGoldenFrame webGolden[] = {
      {{398, -284, 626}, {-1473, 845, 1396}, false, false},
      {{420, -250, 600}, {-1470, 822, 1431}, false, false},
      {{-114, 0, 0}, {-2048, -114, -2048}, false, false},
      {{-23, 0, 0}, {-2048, -23, -2048}, true, false},
      {{0, 1138, 0}, {-1479, 0, -1479}, true, false},
      {{-46, 1138, 0}, {-1479, 46, -1479}, true, false},
      {{-68, 1138, 0}, {-1581, -56, -1376}, false, true},
      {{-68, 1138, 0}, {-1684, -68, -1274}, false, true},
      {{-68, 1138, 0}, {-1786, -68, -1172}, false, true},
      {{-68, 1138, 0}, {-1888, -68, -1069}, false, true},
  };
  WristOrientation::reset(state);
  for (const WebGoldenFrame &frame : webGolden) {
    WristOrientation::xzyToZxz(frame.input, out, state);
    for (int i = 0; i < 3; ++i)
      assert(abs(circularDelta(out[i], frame.output[i])) <= 1);
    assert(state.singular == frame.singular);
    assert(state.recovering == frame.recovering);
  }

  // J5 uses asymmetric soft limits (-87/+112 degrees) and smoothly approaches
  // the corresponding hard limits (-90/+115 degrees).
  WristOrientation::reset(state);
  int16_t limitInput[3] = {(int16_t)-degreeCounts(111), 0, 0};
  WristOrientation::xzyToZxz(limitInput, out, state);
  assert(!state.soft_limited);
  assert(out[1] <= degreeCounts(112));
  limitInput[0] = (int16_t)-degreeCounts(113);
  WristOrientation::xzyToZxz(limitInput, out, state);
  assert(state.soft_limited);
  assert(out[1] > degreeCounts(112));
  assert(out[1] < degreeCounts(115));
  limitInput[0] = (int16_t)-degreeCounts(170);
  WristOrientation::xzyToZxz(limitInput, out, state);
  assert(state.soft_limited);
  assert(out[1] <= degreeCounts(115));

  WristOrientation::reset(state);
  state.initialized = true;
  state.beta = (float)(-86.0 * kPi / 180.0);
  state.reset_frames = 0;
  limitInput[0] = degreeCounts(86);
  WristOrientation::xzyToZxz(limitInput, out, state);
  assert(!state.soft_limited);
  assert(out[1] >= -degreeCounts(87));
  limitInput[0] = degreeCounts(88);
  WristOrientation::xzyToZxz(limitInput, out, state);
  assert(state.soft_limited);
  assert(out[1] < -degreeCounts(87));
  assert(out[1] > -degreeCounts(90));
  limitInput[0] = degreeCounts(170);
  WristOrientation::xzyToZxz(limitInput, out, state);
  assert(state.soft_limited);
  assert(out[1] >= -degreeCounts(90));

  puts("wrist orientation tests: OK");
  return 0;
}
