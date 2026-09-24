// Rendering helpers only. Wrist conversion is performed by the ESP32-S3.
export const degToRad = value => value * Math.PI / 180;
const identity = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

export function multiply(a, b) {
  return a.map((row, r) => row.map((_, c) =>
    a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]));
}

export function rotation(axis, angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  if (axis === "X") return [[1, 0, 0], [0, c, -s], [0, s, c]];
  if (axis === "Y") return [[c, 0, s], [0, 1, 0], [-s, 0, c]];
  return [[c, -s, 0], [s, c, 0], [0, 0, 1]];
}

export function compose(order, angles) {
  return order.split("").reduce((result, axis, i) =>
    multiply(result, rotation(axis, angles[i])), identity());
}

export function maxMatrixError(a, b) {
  let error = 0;
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++)
      error = Math.max(error, Math.abs(a[r][c] - b[r][c]));
  return error;
}
