const Kt = 3.96;
const Ke = 0.414;
const R = 4.3;
const Vbat = 7.4;
const r = 24 / 1000;
const m = 200;
const n = 40 / 8;
const tau = (r, m, a, n) => {
  return (r * m * a) / (2 * n);
};
const duty = (R, tau, Kt, Ke, omega, Vbat) => {
  return ((R * tau) / Kt + Ke * omega) / Vbat;
};

for (let a of [1, 2, 3, 4, 5]) {
  console.log(`a: ${a}, duty: ${duty(R, tau(r, m, a, n), Kt, Ke, 0, Vbat)}`);
}
