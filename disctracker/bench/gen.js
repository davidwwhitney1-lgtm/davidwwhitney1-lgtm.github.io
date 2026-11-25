// gen.js - synthetic throws with exact truth, as the disc's sensor would record them.
// (The author's test bench, dtbuild gen_t.js of 2026-11-09, kept in the repo since 2026-11-23: issue #43.)
//
// A rigid disc: held still, a slow reach-back, a fast pull along a path that curves and then straightens, a wrist
// snap that spins it up in the last few hundredths of a second, release, free flight (drag, lift, torque-free wobble)
// and a net. The IMU sits r off the disc's axis and h above its middle. Sensor model: gyro +-2294 deg/s, accel +-16 g,
// noise, zero errors, the chip's real clock (418.6 samples/s) and ~4 % repeated samples (FIFO slips), all in raw
// counts in DISC axes, as the disc streams them.
(function (G) {
"use strict";
const D2R = Math.PI / 180, R2D = 180 / Math.PI, GMS = 9.80665, TAU = 2 * Math.PI;
const qmul = (a, b) => [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
const qaxis = (ax, ang) => { const s = Math.sin(ang / 2); return [Math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s]; };
const qnorm = q => { const n = Math.hypot(q[0], q[1], q[2], q[3]); return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]; };
const qconj = q => [q[0], -q[1], -q[2], -q[3]];
const qrot = (q, v) => {                     // v_world = q v_body q*
  const [w, x, y, z] = q, tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
};
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]], sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k], dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = a => Math.hypot(a[0], a[1], a[2]);
function rng(seed) { let s = (seed >>> 0) || 1; return () => { s = (s + 0x6D2B79F5) >>> 0; let t = Math.imul(s ^ (s >>> 15), 1 | s); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }

// The six numbers, as throw_metrics.h defines them, from a CoM velocity and an attitude (body -> world).
function sixFrom(q, v, ccw) {
  const horiz = Math.hypot(v[0], v[1]), launch = Math.atan2(v[2], horiz) * R2D, n = qrot(q, [0, 0, 1]);
  let hyzer, nose;
  if (horiz > 0.5) {
    const fx = v[0] / horiz, fy = v[1] / horiz, lx = -fy, ly = fx;
    hyzer = Math.atan2(-(n[0] * lx + n[1] * ly), n[2]) * R2D;
    nose = Math.atan2(-(n[0] * fx + n[1] * fy), n[2]) * R2D - launch;
  } else { hyzer = Math.atan2(-n[1], n[2]) * R2D; nose = Math.atan2(-n[0], n[2]) * R2D; }
  if (!ccw) hyzer = -hyzer;
  return { speed: norm(v) * 2.2369363, launch, hyzer, nose };
}
// Direction of travel relative to where the disc's own x axis pointed (level) when it was last held still: the only
// "heading" the disc can know without a compass.
function relHeading(q0, v) {
  const x0 = qrot(q0, [1, 0, 0]), h0 = Math.atan2(x0[1], x0[0]), hv = Math.atan2(v[1], v[0]);
  let d = (hv - h0) * R2D; while (d > 180) d -= 360; while (d < -180) d += 360; return d;
}

// Giljarhus, Gooding & Njaerheim 2022 (Sports Engineering), CFD, no spin: open data in kegiljarhus/shotshaper.
const WRAITH = {
  dia: 0.211, Jz: 7.465e-3, Jxy: 3.755e-3,
  a: [-90,-80,-70,-60,-50,-40,-30,-20,-15,-10,-8,-6,-4,-2,0,2,4,6,8,10,12,14,16,18,20,25,30,40,50,60,70,80,90],
  cd: [1.0156,1.0413,0.995,0.8904,0.7415,0.6219,0.4648,0.2499,0.1556,0.0769,0.0617,0.0509,0.0432,0.0429,0.0417,0.0578,0.0664,0.0825,0.1058,0.1281,0.1624,0.2015,0.2462,0.2962,0.3591,0.561,0.8141,1.2255,1.2382,1.0793,1.1797,1.199,1.1867],
  cl: [0.0001,-0.2141,-0.4086,-0.5726,-0.6814,-0.7987,-0.8534,-0.694,-0.4917,-0.2137,-0.1607,-0.0952,-0.0401,0.0347,0.1091,0.2013,0.2761,0.3666,0.477,0.5604,0.6684,0.7773,0.888,1.0013,1.1157,1.67,1.8447,1.8161,1.2563,0.7495,0.5236,0.2646,0.0033],
  cm: [-0.0007,-0.0326,-0.0544,-0.0712,-0.085,-0.0985,-0.0953,-0.0728,-0.0742,-0.0966,-0.076,-0.0621,-0.0455,-0.0342,-0.0175,-0.0122,0.0013,0.0128,0.0217,0.0364,0.0486,0.0618,0.0758,0.0925,0.1081,0.1568,0.1712,0.1419,0.0995,0.0731,0.0692,0.0421,0.0016],
};
const FIREBIRD = {
  dia: 0.211, Jz: 7.485e-3, Jxy: 3.759e-3, a: WRAITH.a,
  cd: [0.9642,1.0409,1.0149,0.8892,0.7908,0.6741,0.5343,0.2728,0.1633,0.0813,0.0663,0.0554,0.0474,0.0474,0.0458,0.057,0.0668,0.0846,0.109,0.137,0.1729,0.2182,0.2672,0.3268,0.3924,0.5411,0.7412,1.1114,1.1154,1.0806,1.1862,1.023,1.0295],
  cl: [0.0001,-0.1869,-0.3665,-0.5029,-0.6435,-0.7737,-0.8637,-0.666,-0.4806,-0.29,-0.219,-0.1522,-0.0802,-0.0026,0.0674,0.1646,0.2486,0.3467,0.4523,0.5633,0.6728,0.7904,0.8948,1.0084,1.0936,1.1999,1.3094,1.3222,0.9143,0.6057,0.4181,0.1679,-0.0016],
  cm: [0.0003,-0.0314,-0.047,-0.0638,-0.0787,-0.0892,-0.1007,-0.0753,-0.0805,-0.075,-0.0617,-0.0484,-0.0346,-0.0238,-0.0099,0.002,0.0112,0.0211,0.0323,0.0461,0.0601,0.0749,0.0929,0.1087,0.1159,0.1515,0.1529,0.1355,0.0904,0.0676,0.0749,0.0479,-0.0028],
};
const TABLES = { wraith: WRAITH, firebird: FIREBIRD };
const lerp = (D, key, deg) => { const a = D.a, y = D[key]; deg = Math.max(-90, Math.min(90, deg)); let i = 0; while (i < a.length - 2 && deg > a[i + 1]) i++; const u = (deg - a[i]) / (a[i + 1] - a[i]); return y[i] + u * (y[i + 1] - y[i]); };

function makeThrow(o = {}) {
  const P = Object.assign({
    V: 22, rpm: 900, dir: -1, T: 0.16, accelPeak: 0.6, turn: 70, snap: 0.04, snapShape: "smooth",
    hyzer: 6, nose: -2, launch: 6, rollChange: 10, r: 0.006, rDir: 30, h: 0.005, wobble: 3,
    still: 0.9, reach: 0.4, reachLen: 0.45, grip: null, net: 3.0, rate: 418.6, noiseG: 0.06, noiseA: 0.002,
    biasG: [0.45, -2.5, -0.85], accOff: [0.012, -0.008, 0.03], dup: 0.04, seed: 1, cl: 0.12, cd: 0.08, table: "wraith", spinTauHit: 0.03, afterHit: 0.35,
    iRatio: 1.99,                                               // axial / cross moment of inertia (golf disc ~1.93; thin flat disc 2)
  }, o);
  const R = rng(P.seed);
  if (P.grip == null) P.grip = 360 * R();                    // how the board sits in the hand: the throw's direction in disc axes
  const gauss = () => { let u = 0; while (!u) u = R(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * R()); };
  const H = 1e-4;                                            // fine time step, s
  // P.lead (2026-11-23): seconds of fidgeting before the still hold - a player walking up and settling the grip - so a
  // stream starts with the disc's one-second history already full, as on a disc that has been on for a while. 0 = none.
  const t0 = P.lead || 0;
  const t1 = t0 + P.still, t2 = t1 + P.reach, t3 = t2 + P.T;  // reach-back starts, pull starts, release
  const wRel = P.rpm * TAU / 60;
  // ---- the pull: speed profile, path length, heading
  const ex = Math.log(0.5) / Math.log(P.accelPeak);
  const vOf = u => P.V * (1 - Math.cos(Math.PI * Math.pow(Math.min(1, Math.max(0, u)), ex))) / 2;
  const NP = Math.round(P.T / H);
  const sArr = new Float64Array(NP + 1);
  for (let k = 1; k <= NP; k++) sArr[k] = sArr[k - 1] + 0.5 * (vOf((k - 1) / NP) + vOf(k / NP)) * H;
  const S = sArr[NP], Phi = P.turn * D2R;
  const headOf = s => -P.dir * Phi * (1 - s / S) ** (P.curvePow ?? 2);
  // path in the pull plane: X, Y by integrating the heading over s
  const XY = [[0, 0]];
  for (let k = 1; k <= NP; k++) {
    const ds = sArr[k] - sArr[k - 1], hm = headOf(0.5 * (sArr[k] + sArr[k - 1]));
    XY.push([XY[k - 1][0] + ds * Math.cos(hm), XY[k - 1][1] + ds * Math.sin(hm)]);
  }
  const qLaunch = qaxis([0, 1, 0], -P.launch * D2R);
  const smooth5 = u => u * u * u * (10 - 15 * u + 6 * u * u);       // zero speed and acceleration at both ends
  const snapShape = { smooth: u => u * u * (3 - 2 * u), late: u => u * u * u, early: u => 1 - (1 - u) ** 3, linear: u => u }[P.snapShape];
  // twist (snap) angle: integral of dir * wRel * shape over the last `snap` seconds
  const NS = Math.max(1, Math.round(P.snap / H));
  const twistRate = t => { const u = (t - (t3 - P.snap)) / P.snap; return u <= 0 ? 0 : P.dir * wRel * snapShape(Math.min(1, u)); };
  const twist = new Float64Array(NP + 1); twist[0] = P.grip * D2R;
  for (let k = 1; k <= NP; k++) { const t = t2 + k * H; twist[k] = twist[k - 1] + 0.5 * (twistRate(t - H) + twistRate(t)) * H; }
  // the wobble building up in the hand (wobbleRamp > 0): an extra body turn, its rate s(u) wp (cos phi, sin phi, 0)
  // (built on first use, once qPull0 below exists)
  const NR = Math.round((P.wobbleRamp || 0) / H);
  let qExtra = null;
  const extraAt = k => {
    if (!qExtra) {
      qExtra = new Array(NP + 1);
      P._phRel = TAU * R();
      const wp = P.iRatio * wRel * Math.tan(P.wobble * D2R), kIb = P.iRatio - 1, sm = u => u * u * (3 - 2 * u);
      let qe = [1, 0, 0, 0];
      for (let j = 0; j <= NP; j++) {
        const u = (j - (NP - NR)) / NR;
        if (u > 0) {
          const phi = P._phRel - kIb * (twist[NP] - twist[j]), m = sm(Math.min(1, u)) * wp;
          // turned in world axes, so the rate added in the disc's own axes is exactly this (a turn in body axes would tilt the spin too)
          const wW = qrot(qmul(qe, qPull0(j)), [Math.cos(phi), Math.sin(phi), 0]);
          qe = qnorm(qmul(qaxis(wW, m * H), qe));
        }
        qExtra[j] = qe;
      }
    }
    return qExtra[k];
  };
  const qPull = k => NR > 0 ? qnorm(qmul(extraAt(k), qPull0(k))) : qPull0(k);
  const qPull0 = k => {                                      // attitude at pull step k (before the wobble's own turn)
    const u = k / NP, s = sArr[k], roll = (P.hyzer * (P.dir > 0 ? -1 : 1) + P.rollChange * (1 - smooth5(u))) * D2R;
    return qnorm(qmul(qmul(qmul(qLaunch, qaxis([0, 0, 1], headOf(s))), qmul(qaxis([1, 0, 0], roll), qaxis([0, 1, 0], -P.nose * D2R))), qaxis([0, 0, 1], twist[k])));
  };
  const pPull = k => qrot(qLaunch, [XY[k][0], XY[k][1], 0]);
  // ---- before the pull: held still at the hold point, then a slow reach-back to the pull's start
  const q0p = qPull(0), p0p = pPull(0);
  const d0 = qrot(qLaunch, [Math.cos(headOf(0)), Math.sin(headOf(0)), 0]);
  const pHold = add(p0p, mul(d0, P.reachLen));
  const qHold = qnorm(qmul(q0p, qmul(qaxis([0, 0, 1], -P.dir * (P.windYaw ?? 35) * D2R), qaxis([1, 0, 0], (P.windTilt ?? 8) * D2R))));
  const slerp = (a, b, u) => {
    let c = dot([a[1], a[2], a[3]], [b[1], b[2], b[3]]) + a[0] * b[0];
    if (c < 0) { b = b.map(v => -v); c = -c; }
    if (c > 0.9995) return qnorm(a.map((v, i) => v + (b[i] - v) * u));
    const th = Math.acos(c), s0 = Math.sin((1 - u) * th) / Math.sin(th), s1 = Math.sin(u * th) / Math.sin(th);
    return qnorm(a.map((v, i) => s0 * v + s1 * b[i]));
  };
  // ---- build the fine timeline: position and attitude, through release
  const tl = [];                                              // {t, p, q}
  for (let t = 0; t < t1; t += H) {
    if (t < t0) {                                             // fidgeting: ~4 mm and ~5 deg at 1-3 Hz (gyro 30-90 deg/s, not still)
      // full strength until 60 ms before the hold (a long fade would itself be a still moment), faded in over 0.25 s
      const ri = Math.min(0.25, t0 / 2), ro = Math.min(0.06, t0 / 4);
      const w0 = t < ri ? Math.sin(Math.PI / 2 * t / ri) ** 2 : t > t0 - ro ? Math.sin(Math.PI / 2 * (t0 - t) / ro) ** 2 : 1;
      const fx = w0 * 0.004 * Math.sin(TAU * 2.3 * t), fy = w0 * 0.003 * Math.sin(TAU * 1.7 * t + 0.5), fz = w0 * 0.003 * Math.sin(TAU * 2.9 * t + 1.1);
      const fa = w0 * 5 * D2R * Math.sin(TAU * 2.1 * t), fb = w0 * 4 * D2R * Math.sin(TAU * 1.3 * t + 0.7);
      tl.push({ t, p: add(pHold, [fx, fy, fz]), q: qnorm(qmul(qmul(qHold, qaxis([1, 0, 0], fa)), qaxis([0, 1, 0], fb))) });
      continue;
    }
    const win = Math.sin(Math.PI * (t - t0) / (t1 - t0)) ** 2;   // tremor fades out at both ends: no jumps
    const tr = win * 0.0005 * Math.sin(TAU * 7 * t), ta = win * 0.25 * D2R * Math.sin(TAU * 8.3 * t + 1);
    tl.push({ t, p: add(pHold, [tr, 0.6 * tr, 0]), q: qnorm(qmul(qHold, qaxis([1, 0, 0], ta))) });
  }
  for (let t = t1; t < t2; t += H) { const u = smooth5((t - t1) / P.reach); tl.push({ t, p: add(pHold, mul(sub(p0p, pHold), u)), q: slerp(qHold, q0p, u) }); }
  for (let k = 0; k <= NP; k++) tl.push({ t: t2 + k * H, p: pPull(k), q: qPull(k) });
  // derivatives on the fine timeline: v, a (CoM), w, alpha (body)
  const N0 = tl.length;
  for (let i = 0; i < N0; i++) {
    const a = tl[Math.max(0, i - 1)], b = tl[Math.min(N0 - 1, i + 1)], dt = b.t - a.t;
    tl[i].v = mul(sub(b.p, a.p), 1 / dt);
    let qb = b.q; if (dot([a.q[1], a.q[2], a.q[3]], [qb[1], qb[2], qb[3]]) + a.q[0] * qb[0] < 0) qb = qb.map(v => -v);
    const qd = qmul(qconj(tl[i].q), qb.map((v, k) => (v - a.q[k]) / dt));
    tl[i].w = [2 * qd[1], 2 * qd[2], 2 * qd[3]];
  }
  for (let i = 0; i < N0; i++) {
    const a = tl[Math.max(0, i - 1)], b = tl[Math.min(N0 - 1, i + 1)], dt = b.t - a.t;
    tl[i].acc = mul(sub(b.v, a.v), 1 / dt); tl[i].al = mul(sub(b.w, a.w), 1 / dt);
  }
  // ---- release and free flight: torque-free symmetric disc (I3 = 2 I1), drag, lift, gravity, then the net
  const rel = tl[N0 - 1];
  let p = rel.p.slice(), v = rel.v.slice(), q = rel.q.slice(), w = rel.w.slice();
  if (!(P.wobbleRamp > 0)) {                                  // the old way: the wobble as a step at release
    const wp = P.iRatio * Math.abs(w[2]) * Math.tan(P.wobble * D2R), ph = TAU * R();   // cone half-angle of the spin axis
    w = [w[0] + wp * Math.cos(ph), w[1] + wp * Math.sin(ph), w[2]];
  }
  P._wRel = w.slice();
  const kD = 0.5 * 1.225 * P.cd * 0.057 / 0.175, kL = 0.5 * 1.225 * P.cl * 0.057 / 0.175, g = [0, 0, -GMS];
  const pRel = p.slice();
  let t = t3, impT = null, vImp = null, dImp = null;
  const tauI = 0.06, tEnd = () => impT != null ? impT + tauI + P.afterHit : t + 1;
  const aero = (v, q) => {
    const sp = norm(v); if (sp < 1e-6) return [0, 0, 0];
    const vh = mul(v, 1 / sp), n = qrot(q, [0, 0, 1]);
    let kd = kD, kl = kL;
    if (P.table) {                                            // the table's coefficients at this instant's angle of attack
      const al = -Math.asin(Math.max(-1, Math.min(1, dot(vh, n)))) * R2D, k = 0.5 * (P.rho || 1.225) * Math.PI * 0.1055 * 0.1055 / 0.175, D = TABLES[P.table];
      kd = k * lerp(D, "cd", al); kl = k * lerp(D, "cl", al);
    }
    const drag = mul(vh, -kd * sp * sp);
    const lp = sub(n, mul(vh, dot(n, vh))), ln = norm(lp);
    let aeroF = ln > 1e-9 ? add(drag, mul(lp, kl * sp * sp / ln)) : drag;
    if (P.cs) {                                               // the side force a spinning disc feels, across its path
      const k = 0.5 * (P.rho || 1.225) * Math.PI * 0.1055 * 0.1055 / 0.175, side = cross(n, vh), sl = norm(side), ar = Math.abs(w[2]) * 0.1055 / sp;
      if (sl > 1e-9) aeroF = add(aeroF, mul(side, Math.sign(w[2]) * k * P.cs * ar * sp * sp / sl));
    }
    return aeroF;
  };
  while (t < tEnd() && t < t3 + 3) {
    let acc;
    if (impT == null && norm(sub(p, pRel)) >= P.net) { impT = t; vImp = v.slice(); dImp = mul(v, 1 / norm(v)); }
    if (impT == null) acc = add(g, aero(v, q));
    else if (t < impT + tauI) acc = mul(dImp, -(norm(vImp) / tauI) * (Math.PI / 2) * Math.sin(Math.PI * (t - impT) / tauI));
    else { acc = [0, 0, 0]; v = [0, 0, 0]; }
    // body rates: Euler's equations; spin and wobble stop fast in the net
    const kI = P.iRatio - 1;                                // Euler, symmetric body: the wobble turns at kI * spin in the disc's axes
    let al = [-kI * w[1] * w[2], kI * w[0] * w[2], -w[2] / 25];
    if (impT != null) al = mul(w, -1 / P.spinTauHit);
    tl.push({ t, p: p.slice(), q: q.slice(), v: v.slice(), w: w.slice(), acc: impT != null && t >= impT + tauI ? [0, 0, 0] : acc, al });
    // integrate (small steps)
    v = add(v, mul(acc, H)); p = add(p, mul(v, H));
    if (impT == null) { const ang = kI * w[2] * H, cg = Math.cos(ang), sg = Math.sin(ang); w = [w[0] * cg - w[1] * sg, w[0] * sg + w[1] * cg, w[2] + al[2] * H]; }   // rotation exact
    else w = add(w, mul(al, H));
    if (impT == null && P.wobbleTau > 0) { const d = Math.exp(-H / P.wobbleTau); w = [w[0] * d, w[1] * d, w[2]]; }   // wobble dying away (aerodynamic damping)
    const wn = norm(w);
    if (wn > 1e-12) q = qnorm(qmul(q, qaxis(mul(w, 1 / wn), wn * H)));
    t += H;
  }
  // ---- sample like the sensor
  const rB = [P.r * Math.cos(P.rDir * D2R), P.r * Math.sin(P.rDir * D2R), P.h];
  const nS = Math.floor(tl[tl.length - 1].t * P.rate);
  const raw = new Int16Array(nS * 6), rawU = new Float64Array(nS * 6), clamp = x => Math.max(-32768, Math.min(32767, Math.round(x)));
  const speedAll = new Float32Array(nS);
  let prev = null, prevU = null;
  const at = tk => {                                         // the fine-timeline entry nearest tk
    let lo = 0, hi = tl.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tl[m].t <= tk) lo = m; else hi = m; }
    return tk - tl[lo].t <= tl[hi].t - tk ? tl[lo] : tl[hi];
  };
  const mt = (P.mountTilt || 0) * D2R, md = (P.mountDir ?? 40) * D2R, qM = mt ? qaxis([Math.cos(md), Math.sin(md), 0], mt) : null;
  const toSensor = v => qM ? qrot(qM, v) : v;
  const scaled = (v, e) => e ? [v[0] * (1 + e[0]) + e[3] * v[1], v[1] * (1 + e[1]) + e[3] * v[2], v[2] * (1 + e[2]) + e[3] * v[0]] : v;
  const forceAt = s => {
    let fB = add(add(qrot(qconj(s.q), sub(s.acc, g)), cross(s.al, rB)), cross(s.w, cross(s.w, rB)));
    if (P.ring && s.t >= t3) { const a = P.ring.g * GMS * Math.exp(-(s.t - t3) / P.ring.tau) * Math.sin(TAU * P.ring.hz * (s.t - t3)); fB = add(fB, [a, 0.6 * a, 0.5 * a]); }
    return scaled(toSensor(fB), P.accErr).map((x, i) => x / GMS + P.accOff[i]);
  };
  const rateAt = s => scaled(toSensor(s.w), P.gyroErr).map((x, i) => x * R2D + P.biasG[i]);
  if (P.filter) {                                             // 2-pole low-pass, 10 kHz fine steps: force clipped, then filtered
    const biquad = fc => { const w0 = TAU * fc * H, c = Math.cos(w0), al = Math.sin(w0) / Math.SQRT2, a0 = 1 + al; return { b0: (1 - c) / 2 / a0, b1: (1 - c) / a0, b2: (1 - c) / 2 / a0, a1: -2 * c / a0, a2: (1 - al) / a0, x1: 0, x2: 0, y1: 0, y2: 0 }; };
    const stepF = (f, x) => { const y = f.b0 * x + f.b1 * f.x1 + f.b2 * f.x2 - f.a1 * f.y1 - f.a2 * f.y2; f.x2 = f.x1; f.x1 = x; f.y2 = f.y1; f.y1 = y; return y; };
    const fA = [0, 1, 2].map(() => biquad(P.filter.accHz ?? 208)), fG = [0, 1, 2].map(() => biquad(P.filter.gyroHz ?? 136));
    for (const s of tl) {
      const a = forceAt(s), r = rateAt(s);
      s.aF = a.map((x, i) => stepF(fA[i], Math.max(-16, Math.min(16, x))));
      s.gF = r.map((x, i) => stepF(fG[i], x));
    }
  }
  for (let k = 0; k < nS; k++) {
    const s = at(k / P.rate);
    const aS = P.filter ? s.aF : forceAt(s), gS = P.filter ? s.gF : rateAt(s);
    let u = [0, 1, 2].map(i => (gS[i] + P.noiseG * gauss()) / 0.07)
      .concat([0, 1, 2].map(i => (aS[i] + P.noiseA * gauss()) / 0.000488));
    let c = u.map(clamp);
    if (prev && R() < P.dup) { c = prev.slice(); u = prevU.slice(); }   // a FIFO slip: the previous set again
    raw.set(c, k * 6); rawU.set(u, k * 6); prev = c; prevU = u;
    speedAll[k] = s.v ? norm(s.v) : 0;                        // the disc's true speed at each reading (lab.html draws it)
  }
  // ---- what the disc keeps: from 415 samples before the trigger to just after the impact (or 0.28 s of flight)
  let trig = -1;
  for (let k = Math.round((t0 + P.still * 0.5) * P.rate); k < nS; k++) {
    const a = Math.hypot(raw[k * 6 + 3], raw[k * 6 + 4], raw[k * 6 + 5]) * 0.000488, gm = Math.hypot(raw[k * 6], raw[k * 6 + 1], raw[k * 6 + 2]) * 0.07;
    if (a >= 3.5 || gm >= 600) { trig = k; break; }
  }
  const kRel = t3 * P.rate, kImp = impT != null ? impT * P.rate : null;
  const start = Math.max(0, trig - 415), end = Math.min(nS, kImp != null ? Math.ceil(kImp) + 6 : Math.ceil(kRel + 128));
  const cap = raw.slice(start * 6, end * 6), capU = rawU.slice(start * 6, end * 6);   // capU: what an unlimited sensor would read
  // ---- truth at release
  const ccw = P.dir > 0, six = sixFrom(rel.q, rel.v, ccw);
  // the disc's real axis: the middle of the circle its axis traces in flight
  const wR = P._wRel, sgW = wR[2] < 0 ? -1 : 1, Ln = Math.hypot(wR[0], wR[1], P.iRatio * wR[2]);
  const nL = qrot(rel.q, [sgW * wR[0] / Ln, sgW * wR[1] / Ln, sgW * P.iRatio * wR[2] / Ln]);
  { const v = rel.v, hz = Math.hypot(v[0], v[1]), lc = Math.atan2(v[2], hz) * R2D, fx = v[0] / hz, fy = v[1] / hz;
    six.noseAxis = Math.atan2(-(nL[0] * fx + nL[1] * fy), nL[2]) * R2D - lc;
    six.hyzerAxis = (ccw ? 1 : -1) * Math.atan2(-(nL[0] * -fy + nL[1] * fx), nL[2]) * R2D; }
  const truth = Object.assign(six, { speedAll, spin: P.rpm, heading: relHeading(qHold, rel.v), kRel: kRel - start, kImp: kImp != null ? kImp - start : null,
    vRel: rel.v, qRel: rel.q, qHold, peakG: 0 });
  // truth path (CoM) per captured sample, for drawing
  truth.path = [];
  for (let k = start; k < end; k++) { const s = at(k / P.rate); truth.path.push(s.p); }
  for (let k = start; k < end; k++) { const a = Math.hypot(...sub(at(k / P.rate).acc, g)) / GMS; if (a > truth.peakG) truth.peakG = a; }
  return { P, raw: cap, rawU: capU, rawAll: raw, rawUAll: rawU, n: end - start, start, trig: trig - start, truth, rate: P.rate,
           qEnd: tl[tl.length - 1].q, t2, t3 };
}

G.DiscGen = { makeThrow, sixFrom, relHeading, qrot, qmul, qaxis, qconj, qnorm };
})(typeof window !== "undefined" ? window : globalThis);
