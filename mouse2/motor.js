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


const Vs = 200
const Vmax = 600
const Ve = 200
const d = 400
const a = 400
const freq = 1000
function motion(start_velocity,max_velocity,end_velocity,distance,accel,frequency) {
  let t1 = (max_velocity - start_velocity) * frequency / accel;
  let t3 = (max_velocity - end_velocity) * frequency / accel;
  const l1 = t1 * (start_velocity + max_velocity) / (2.0 * frequency);
  const l3 = t3 * (max_velocity + end_velocity) / (2.0 * frequency);
  let t2 = (distance - l1 - l3) * frequency / max_velocity;
  if (t1 > t2) {
    const Vmaxp = Math.sqrt(((start_velocity * start_velocity + end_velocity * end_velocity) + 2 * accel * distance) / 2)
    t1 = (Vmaxp - start_velocity) * frequency / accel;
    t3 = (Vmaxp - end_velocity) * frequency / accel;
    t2 = 0
  }
  motion = {}
  motion.t1 = t1;
  motion.t2 = t1 + t2;
  motion.t3 = t1 + t2 + t3;
  motion.start_velocity = start_velocity;
  motion.max_velocity = max_velocity;
  motion.end_velocity = end_velocity;
  motion.distance = distance;
  motion.delta = 1.0 / frequency;
  motion.accel = accel;
  return motion
}

console.log(motion(Vs, Vmax, Ve, d, a, freq));
