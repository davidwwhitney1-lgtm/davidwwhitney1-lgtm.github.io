// handling.js - simulated handling that is NOT a throw, as the disc's sensor would record it, for testing the trigger
// and counting rules: picking the disc up and setting it down, a shake to wake it, flipping it over, walking with it,
// dropping it, a pump fake, spinning it in the hand, a toss caught in the hand, rolling on its edge after a landing,
// and falling out of the net after a throw.
//
// Each motion is built from the disc's acceleration (world axes) and its rotation rate (disc axes), added up in small
// steps, so the readings are physically consistent. The sensor model is gen.js's: 4.3 mm off the axis and 5 mm above
// the middle, noise, zero errors, the chip's clock (418.6 a second), repeated samples (FIFO slips), and the limits
// (+-16 g, +-2294 deg/s). Needs gen.js (DiscGen) for the quaternion helpers.
//
// DiscHandling.make(kind, o) -> { raw, rawU, n, rate, kind, P, events: [{t, what}], qEnd }
(function (G) {
"use strict";
const { qmul, qaxis, qconj, qnorm, qrot } = G.DiscGen;
const D2R = Math.PI / 180, R2D = 180 / Math.PI, GMS = 9.80665, TAU = 2 * Math.PI;
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k], norm = a => Math.hypot(a[0], a[1], a[2]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function rng(seed) { let s = (seed >>> 0) || 1; return () => { s = (s + 0x6D2B79F5) >>> 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
// smooth step 0 -> 1 with zero speed and acceleration at both ends, and its first two derivatives
const s5 = u => u <= 0 ? 0 : u >= 1 ? 1 : u * u * u * (10 - 15 * u + 6 * u * u);
const s5d = u => u <= 0 || u >= 1 ? 0 : 30 * u * u * (1 - u) * (1 - u);
const s5dd = u => u <= 0 || u >= 1 ? 0 : 60 * u * (1 - u) * (1 - 2 * u);
const H = 2e-4;                                              // step, s (5000 a second: an impact of 10 ms gets 50 steps)

// The timeline: segments in order, each { T, acc(t, st) world m/s^2 (gravity NOT included: a held disc has acc 0),
// w(t, st) disc-axes rad/s, free: true while nothing holds the disc (gravity then acts) }.
function integrate(segs, q0, p0, v0) {
  const tl = []; let t = 0, p = p0.slice(), v = (v0 || [0, 0, 0]).slice(), q = q0.slice();
  for (const sg of segs) {
    const n = Math.round(sg.T / H);
    for (let i = 0; i < n; i++) {
      const lt = i * H, st = { p, v, q, t: lt };
      let acc = sg.acc ? sg.acc(lt, st) : [0, 0, 0];
      if (sg.free) acc = add(acc, [0, 0, -GMS]);
      const w = sg.w ? sg.w(lt, st) : [0, 0, 0];
      tl.push({ t, p: p.slice(), v: v.slice(), q: q.slice(), acc, w });
      v = add(v, mul(acc, H)); p = add(p, mul(v, H));
      const wn = norm(w); if (wn > 1e-12) q = qnorm(qmul(q, qaxis(mul(w, 1 / wn), wn * H)));
      t += H;
    }
  }
  for (let i = 0; i < tl.length; i++) {                       // angular acceleration (disc axes)
    const a = tl[Math.max(0, i - 1)], b = tl[Math.min(tl.length - 1, i + 1)];
    tl[i].al = mul(sub(b.w, a.w), 1 / Math.max(H, b.t - a.t));
  }
  return tl;
}

// Sample the timeline like the sensor (gen.js's model).
function sample(tl, P, R) {
  const gauss = () => { let u = 0; while (!u) u = R(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * R()); };
  const rB = [P.r * Math.cos(P.rDir * D2R), P.r * Math.sin(P.rDir * D2R), P.h];
  const T = tl[tl.length - 1].t, nS = Math.floor(T * P.rate);
  const raw = new Int16Array(nS * 6), rawU = new Float64Array(nS * 6), clamp = x => Math.max(-32768, Math.min(32767, Math.round(x)));
  let prev = null, prevU = null;
  for (let k = 0; k < nS; k++) {
    const s = tl[Math.min(tl.length - 1, Math.round(k / P.rate / H))];
    const fB = add(add(qrot(qconj(s.q), add(s.acc, [0, 0, GMS])), cross(s.al, rB)), cross(s.w, cross(s.w, rB)));
    let u = [0, 1, 2].map(i => (s.w[i] * R2D + P.biasG[i] + P.noiseG * gauss()) / 0.07)
      .concat([0, 1, 2].map(i => (fB[i] / GMS + P.accOff[i] + P.noiseA * gauss()) / 0.000488));
    let c = u.map(clamp);
    if (prev && R() < P.dup) { c = prev.slice(); u = prevU.slice(); }
    raw.set(c, k * 6); rawU.set(u, k * 6); prev = c; prevU = u;
  }
  return { raw, rawU, n: nS };
}

// pieces of motion
const rest = T => ({ T });
const tremor = (T, R) => {                                   // a hand holding the disc: a little tremor, fading in and out
  const f1 = 6 + 3 * R(), f2 = 7 + 3 * R(), ph = TAU * R(), A = 0.0005, W = 0.3 * D2R;
  return { T, acc: t => { const e = Math.sin(Math.PI * t / T) ** 2, w1 = TAU * f1; return [-A * w1 * w1 * Math.sin(w1 * t) * e, -0.6 * A * w1 * w1 * Math.sin(w1 * t + ph) * e, 0]; },
    w: t => { const e = Math.sin(Math.PI * t / T) ** 2, w2 = TAU * f2; return [W * w2 * Math.cos(w2 * t) * e, 0.7 * W * w2 * Math.cos(w2 * t + ph) * e, 0]; } };
};
const move = (T, d) => ({ T, acc: t => mul(d, s5dd(t / T) / (T * T)) });                   // straight line, d metres
const turn = (T, axis, deg) => ({ T, w: t => mul(axis, deg * D2R * s5d(t / T) / T) });    // about a disc axis
const moveTurn = (T, d, axis, deg) => ({ T, acc: move(T, d).acc, w: turn(T, axis, deg).w });
// an oscillation: position A sin(w t) along d, and angle Th sin(w t) about a disc axis, both in a sin^2 envelope
const oscillate = (T, f, A, d, Th, axis) => {
  const w = TAU * f, e = t => Math.sin(Math.PI * t / T) ** 2, ed = t => Math.PI / T * Math.sin(TAU * t / T), edd = t => 2 * (Math.PI / T) ** 2 * Math.cos(TAU * t / T);
  return { T, acc: t => mul(d, A * (edd(t) * Math.sin(w * t) + 2 * ed(t) * w * Math.cos(w * t) - e(t) * w * w * Math.sin(w * t))),
    w: t => mul(axis, Th * (ed(t) * Math.sin(w * t) + e(t) * w * Math.cos(w * t))) };
};
// falling until it lands: the ground (z = 0) is a spring and damper that only pushes; the spin dies away on it
const fall = (T, k, c, spinTau) => ({ T, free: true,
  acc: (t, st) => { const pen = -st.p[2]; return pen > 0 ? [-st.v[0] * 8, -st.v[1] * 8, Math.max(0, k * pen - c * st.v[2])] : [0, 0, 0]; },
  w: (t, st) => { const on = st.p[2] <= 0.002; return on ? mul(st.w0 || [0, 0, 0], Math.exp(-t / spinTau)) : st.w0 || [0, 0, 0]; } });

const qFromEuler = (yaw, pitch, roll) => qnorm(qmul(qmul(qaxis([0, 0, 1], yaw * D2R), qaxis([0, 1, 0], pitch * D2R)), qaxis([1, 0, 0], roll * D2R)));

function make(kind, o = {}) {
  const P = Object.assign({ rate: 418.6, r: 0.0043, rDir: 30, h: 0.005, noiseG: 0.06, noiseA: 0.002, biasG: [0.45, -2.5, -0.85],
    accOff: [0.012, -0.008, 0.03], dup: 0.04, seed: 1, lead: 1.6 }, o);
  const R = rng(P.seed), rr = (a, b) => a + (b - a) * R();
  const events = [], segs = [];
  let q0 = qFromEuler(rr(0, 360), 0, 0), p0 = [0, 0, 0.75];   // flat, label up, on a table (z: table height)
  const held = () => qFromEuler(rr(0, 360), rr(-25, 10), rr(-35, 35));   // how a hand holds a disc
  let t = 0, vStart = null;
  const push = (sg, what) => { if (what) events.push({ t, what }); segs.push(sg); t += sg.T; };
  switch (kind) {
    case "pickup": {                                          // lift off a table, hold, set it back down
      const lift = rr(0.3, 0.6), T = rr(0.45, 0.9), tilt = rr(25, 70);
      push(rest(P.lead), "lying still");
      push(moveTurn(T, [rr(-0.2, 0.2), rr(-0.2, 0.2), lift], [1, 0, 0], tilt), "picked up");
      push(tremor(rr(1, 2), R), "held");
      push(moveTurn(T, [0, 0, 0], [1, 0, 0], -tilt), null);
      push(moveTurn(T * 1.1, [0, 0, -lift], [0, 0, 1], rr(-40, 40)), "set down");
      push(rest(1.2), "lying still");
      break;
    }
    case "shake": {                                           // a shake to wake the disc
      q0 = held(); const f = rr(4, 7), A = rr(0.02, 0.04), T = rr(0.8, 1.6), d = [Math.cos(rr(0, TAU)), Math.sin(rr(0, TAU)), rr(-0.3, 0.6)];
      push(tremor(P.lead, R), "held");
      push(oscillate(T, f, A, mul(d, 1 / norm(d)), rr(8, 25) * D2R, [Math.cos(rr(0, TAU)), Math.sin(rr(0, TAU)), 0]), "shaken");
      push(tremor(1.5, R), "held");
      break;
    }
    case "flip": {                                            // turned over in the hand, and back
      q0 = held(); const T = rr(0.25, 0.5);
      push(tremor(P.lead, R), "held");
      push(turn(T, [1, 0, 0], 180), "flipped over");
      push(tremor(rr(0.6, 1.2), R), "held");
      push(turn(T, [1, 0, 0], -180), "flipped back");
      push(tremor(1.2, R), "held");
      break;
    }
    case "walk": {                                            // carried while walking
      q0 = held(); const f = rr(1.6, 2.1), T = rr(5, 8);
      push(tremor(P.lead, R), "held");
      const bob = rr(0.015, 0.03), sway = rr(0.005, 0.015), e = t => Math.sin(Math.PI * t / T) ** 2;
      // a heel strike each step: a short upward push of about half a g
      const heel = t => { const ph = (t * f) % 1; return ph < 0.06 ? 5 * Math.sin(Math.PI * ph / 0.06) ** 2 : 0; };
      push({ T, acc: t => [0, -sway * (TAU * f / 2) ** 2 * Math.sin(TAU * f / 2 * t) * e(t), (-bob * (TAU * f) ** 2 * Math.sin(TAU * f * t) + heel(t) - 5 * 0.03) * e(t)],
             w: t => [3 * D2R * TAU * f / 2 * Math.cos(TAU * f / 2 * t) * e(t), 2 * D2R * TAU * f * Math.cos(TAU * f * t) * e(t), 0] }, "walking");
      push(tremor(1.2, R), "stopped");
      break;
    }
    case "drop": {                                            // dropped onto grass or a floor
      q0 = qFromEuler(rr(0, 360), rr(-30, 30), rr(-30, 30)); p0 = [0, 0, rr(0.7, 1.3)];
      const contact = rr(0.012, 0.04), k = (Math.PI / contact) ** 2, c = 2 * rr(0.35, 0.6) * Math.PI / contact;
      push(tremor(P.lead, R), "held");
      const w0 = [rr(-1.5, 1.5), rr(-1.5, 1.5), rr(-1, 1)], fl = fall(rr(1.2, 1.8), k, c, 0.05);
      const wfl = fl.w; fl.w = (tt, st) => { st.w0 = w0; return wfl(tt, st); };
      push(fl, "let go");
      break;
    }
    case "pumpfake": {                                        // a reach-back and a fast pull, but the disc stays in the hand
      q0 = held(); const d = rr(0.45, 0.75), T = rr(0.15, 0.24), back = rr(0.3, 0.5);
      push(tremor(P.lead, R), "held");
      push(moveTurn(back, [-d * 0.8, rr(-0.1, 0.1), 0], [0, 0, 1], rr(20, 40)), "reach back");
      push(moveTurn(T, [d, rr(-0.1, 0.1), rr(-0.05, 0.1)], [0, 0, 1], -rr(30, 70)), "pull, held on to");
      push(moveTurn(0.6, [-d * 0.2, 0, 0], [0, 0, 1], rr(-10, 10)), null);
      push(tremor(1.2, R), "held");
      break;
    }
    case "handspin": {                                        // spun about its axis in the fingers, then stopped
      q0 = qFromEuler(rr(0, 360), rr(-8, 8), rr(-8, 8)); const rpm = rr(150, 450) * (R() < 0.5 ? -1 : 1), up = rr(0.15, 0.3), hold = rr(0.5, 1.2);
      const wz = rpm * TAU / 60;
      push(tremor(P.lead, R), "held flat");
      push({ T: up, w: t => [0, 0, wz * s5(t / up)] }, "spun up");
      push({ T: hold, w: () => [0, 0, wz] }, "spinning in the hand");
      push({ T: 0.12, w: t => [0, 0, wz * (1 - s5(t / 0.12))] }, "stopped");
      push(tremor(1.2, R), "held");
      break;
    }
    case "toss": {                                            // tossed straight up, flat, with some spin, and caught
      q0 = qFromEuler(rr(0, 360), rr(-10, 10), rr(-10, 10)); const v0 = rr(1.8, 2.8), Tu = rr(0.12, 0.2), rpm = (o.rpm ?? rr(60, 450)) * (R() < 0.5 ? -1 : 1);
      const wz = rpm * TAU / 60, flight = 2 * v0 / GMS, Tc = rr(0.05, 0.1);
      push(tremor(P.lead, R), "held flat");
      push({ T: Tu, acc: t => [0, 0, v0 * s5d(t / Tu) / Tu], w: t => [0, 0, wz * s5(t / Tu)] }, "tossed up");
      push({ T: flight, free: true, w: () => [0, 0, wz] }, "in the air");
      push({ T: Tc, acc: t => [0, 0, v0 * s5d(t / Tc) / Tc], w: t => [0, 0, wz * (1 - s5(t / Tc))] }, "caught");
      push(tremor(1.2, R), "held");
      break;
    }
    case "roll": {                                            // rolling on its edge after a landing, slowing down
      const R0 = 0.105, v0 = rr(1.5, 3.5), T = rr(1.2, 2.5), w0 = v0 / R0 * (R() < 0.5 ? -1 : 1);
      const side = R() < 0.5 ? 1 : -1, vz = rr(2, 4.5), Tl = 0.05;
      q0 = qnorm(qmul(qaxis([0, 0, 1], rr(0, TAU)), qaxis([1, 0, 0], Math.PI / 2 * side))); p0 = [0, 0, R0];
      push(rest(P.lead), "on its edge");
      vStart = [v0, 0, -vz];
      push({ T: Tl, acc: t => [0, 0, vz * s5d(t / Tl) / Tl], w: t => [0, 0, w0 * s5(t / Tl)] }, "lands on its edge");
      push({ T, acc: t => [-v0 / T + 0.8 * Math.sin(TAU * 13 * t), 0, 2.5 * Math.sin(TAU * 9 * t) * Math.exp(-t)], w: t => [0, 0, w0 * (1 - t / T)] }, "rolling");
      push(moveTurn(0.4, [0, 0, -0.09], [1, 0, 0], -90 * side), "falls flat");
      push(rest(1.5), "lying still");
      break;
    }
    case "netdrop": {                                         // after a throw into the net: hangs, then falls out
      q0 = o.q0 || qFromEuler(rr(0, 360), rr(-50, 50), rr(-50, 50)); p0 = [0, 0, rr(0.5, 0.9)];
      const contact = rr(0.012, 0.035), k = (Math.PI / contact) ** 2;
      push(rest(o.hang ?? rr(0.3, 1.5)), "hanging in the net");
      const w0 = [rr(-2, 2), rr(-2, 2), rr(-3, 3)], fl = fall(rr(1.2, 1.6), k, 2 * rr(0.35, 0.6) * Math.PI / contact, 0.05), wfl = fl.w;
      fl.w = (tt, st) => { st.w0 = w0; return wfl(tt, st); };
      push(fl, "falls out of the net");
      break;
    }
    default: throw new Error(`unknown handling "${kind}"`);
  }
  push(kind === "drop" || kind === "netdrop" || kind === "roll" || kind === "pickup" ? rest(3.5) : tremor(3.5, R), null);
  const tl = integrate(segs, q0, p0, vStart);
  const s = sample(tl, P, R);
  return Object.assign(s, { rate: P.rate, kind, P, events, qEnd: tl[tl.length - 1].q });
}
const KINDS = ["pickup", "shake", "flip", "walk", "drop", "pumpfake", "handspin", "toss", "roll", "netdrop"];
const NAMES = { pickup: "Picked up and set down", shake: "Shaken to wake it", flip: "Flipped over in the hand", walk: "Carried while walking",
  drop: "Dropped on the ground", pumpfake: "Pump fake (pulled but not let go)", handspin: "Spun in the fingers", toss: "Tossed up and caught",
  roll: "Rolling on its edge after a landing", netdrop: "Falling out of the net" };
G.DiscHandling = { make, KINDS, NAMES, integrate, sample };
})(typeof window !== "undefined" ? window : globalThis);
