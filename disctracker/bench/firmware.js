// firmware.js - the disc's live loop, in JavaScript, sample by sample:
//   feedSample()    the trigger, the capture, the release found live, and the end of the capture (impact, flight
//                   window full, 3 s limit);
//   finalizeThrow() counted or not, the hard-pull rule, quiet captures, the refractory time and the re-arm;
//   tailFeed()      the rest of the flight after a counted throw, up to the landing or 10 s.
// All of it from firmware/disc_tracker_v2/disc_tracker_v2.ino and throw_metrics.h, for log version 5 (firmware v5,
// on the trackers in November 2026) and log versions 6 and 7 (main). Each capture is analysed with sim.html's
// analyzeCapture (DiscSimCode), the JavaScript copy of throw_metrics.h's analyze() (rule R5).
//
// With it, any raw stream (a recording from sim.html, or a simulated one from gen.js) can be run through the disc's
// own decisions on a computer, and every decision can be drawn: lab.html does that, and so do the bench's studies.
//
// DiscFirmware.run(S, cfg): S = raw counts, n x 6 (gx gy gz ax ay az, disc axes) at the disc's sample rate.
// cfg: { an (DiscSimCode build), ver (5, 6 or 7), level (1-5), alsb, glsb, bias [x, y, z] deg/s (the disc's stored
// gyro offset), radius (m, 0 = not known), locked (radius set by the calibration page), rateHz (measured, 0 = 416),
// gap (optional Uint8Array: 1 = samples were lost just before this one) }.
// Returns { captures, state, relCode, impForce, landDev, cfg }. The arrays have one value per sample of S:
//   state:    STATE codes below;
//   relCode:  why the live release test failed at that sample (REL codes), while it was being tested;
//   impForce: the force the live impact test compared with 4 g (steady part removed), in flight;
//   landDev:  how far a reading was from the readings before it, in the flight tail (the landing test).
(function (G) {
"use strict";
// SENS[] in disc_tracker_v2.ino: trigger, release and counting rules of each sensitivity level.
const SENS = [null,
  { trigG: 1.4, trigDps: 120, minPeakG: 1.4, minSpinRpm: 0, freeG: 1.25, relMinRpm: 0, relPeakG: 1.4, allowNoRelease: true },
  { trigG: 2.0, trigDps: 250, minPeakG: 2.0, minSpinRpm: 15, freeG: 1.6, relMinRpm: 40, relPeakG: 1.5, allowNoRelease: true },
  { trigG: 2.5, trigDps: 400, minPeakG: 2.0, minSpinRpm: 30, freeG: 3.0, relMinRpm: 100, relPeakG: 2.0, allowNoRelease: false },
  { trigG: 3.0, trigDps: 500, minPeakG: 2.0, minSpinRpm: 45, freeG: 3.0, relMinRpm: 150, relPeakG: 2.0, allowNoRelease: false },
  { trigG: 3.5, trigDps: 600, minPeakG: 2.0, minSpinRpm: 60, freeG: 3.0, relMinRpm: 150, relPeakG: 2.0, allowNoRelease: false }];
// The firmware's constants, in samples of the nominal 416 a second (they are counts in the firmware, not times).
const K = {
  ODR: 416, PRE: 416, MAXS: 1248, SETTLE: 12, FLIGHT: 104, HOLD: 10, WIN: 36, GAP: 20, STEADY_N: 20, PRE_PEAK: 208,
  IMPACT_G: 4.0, IMPACT_TRIM: 5, IMPACT_BIG_G: 12, REL_HARD_PEAK_G: 12, REL_HARD_G: 5,
  HARD_PULL_G: 8, HARD_SPIN_DPS: 1000, HARD_SPIN_N: 40, QUIET_MAX_G: 6,
  REFRACTORY_MS: 300, REARM_STILL_N: 50, REARM_MAX_MS: 8000, REARM_DPS: 200, REARM_G: 0.5,
  TAIL_MAX: 4160, TAIL_AFTER: 104, LAND_G: 4.5, LAND_BIG_G: 12, LAND_JUMP_G: 1.2, LAND_BIG_JUMP: 3, LAND_HIST: 36, LAND_CONFIRM: 40,
  SAT: 32200, MED_MAX: 64, BIAS_TOL: 1.5,
};
// What the disc is doing at each sample.
const STATE = { FILLING: 1, ARMED: 2, REFRACTORY: 3, SETTLING: 4, CAPTURE: 5, FLIGHT: 6, TAIL: 7, LOST: 8 };
// Why the live release test failed at a candidate sample (throw_metrics.h releaseAt, in its order); 0 = released.
const REL = { OK: 0, PEAK: 1, HOLD: 2, FIT: 3, GAP: 4, SPIN: 5 };
const WHY = { COUNTED: 0, PEAK: 1, SPIN: 2, NO_RELEASE: 3, LOST: 4, HANDLING: 5 };
const END = { IMPACT: 1, WINDOW: 2, LIMIT: 3, LOST: 4 };
const D2R = Math.PI / 180, GMS = 9.80665;

function medianOf(v) {
  const a = Float64Array.from(v).sort(), m = a.length;
  return !m ? 0 : (m & 1 ? a[m >> 1] : 0.5 * (a[m / 2 - 1] + a[m / 2]));
}

function run(S, cfg) {
  const n = S.length / 6, ver = cfg.ver || 7, L = SENS[cfg.level || 5] || SENS[5], v6 = ver >= 6;
  const AL = cfg.alsb || 0.000488, GL = cfg.glsb || 0.07, bias = cfg.bias || [0, 0, 0];
  let radius = cfg.radius > 0.0005 && cfg.radius < 0.03 ? cfg.radius : 0;
  const an = cfg.an;
  const ax = i => S[i * 6 + 3] * AL, ay = i => S[i * 6 + 4] * AL, az = i => S[i * 6 + 5] * AL, gz = i => S[i * 6 + 2] * GL;
  const mag = i => Math.hypot(ax(i), ay(i), az(i));
  const rot = i => Math.hypot(S[i * 6] * GL, S[i * 6 + 1] * GL, S[i * 6 + 2] * GL);
  const inRange = 0.98 * 32767 * GL;
  const sat = v => v >= K.SAT || v <= -K.SAT;
  const state = new Uint8Array(n), relCode = new Int8Array(n).fill(-1);
  const impForce = new Float32Array(n).fill(NaN), landDev = new Float32Array(n).fill(NaN);
  const captures = [];
  const refrN = Math.floor(K.REFRACTORY_MS * K.ODR / 1000);    // 124 samples: integer maths, as the firmware
  const rearmMaxN = Math.floor(K.REARM_MAX_MS * K.ODR / 1000);

  // ---- throw_metrics.h's live helpers, on stream indices (a capture is a contiguous stretch of the stream)
  const inMed = (a, b) => { const xs = [], ys = []; for (let k = a; k < b && xs.length < K.MED_MAX; k++) { xs.push(ax(k)); ys.push(ay(k)); } return [medianOf(xs), medianOf(ys)]; };
  const zMed = (a, b) => { const zs = []; for (let k = a; k < b && zs.length < K.MED_MAX; k++) zs.push(az(k)); return medianOf(zs); };
  const spinLevel = (a, b) => { const zs = []; for (let k = a; k < b && zs.length < K.MED_MAX; k++) zs.push(Math.abs(gz(k))); return medianOf(zs); };
  const comp = (k, mx, my, mz) => Math.hypot(ax(k) - mx, ay(k) - my, az(k) - mz);
  // releaseAt(capture, i, n, pullPeak) with i and n as stream indices; returns the REL code of the first failed test.
  // Version 5 (before log version 6): no axial (lift) part and no hard-pull limits.
  const releaseAt = (i, end, peak) => {
    if (i + K.HOLD > end) return REL.HOLD;
    const e = Math.min(i + K.WIN, end), [mx, my] = inMed(i, e), mz = v6 ? zMed(i, e) : 0;
    const hard = v6 && peak >= K.REL_HARD_PEAK_G;
    const holdLim = hard ? K.REL_HARD_G : L.freeG, gapLim = hard ? K.REL_HARD_G + 0.5 : L.trigG;
    for (let k = i; k < i + K.HOLD; k++) if (comp(k, mx, my, mz) > holdLim) return REL.HOLD;
    let gzMax = 0, gzSum = 0, hMax = 0, hSum = 0;
    for (let k = i; k < e; k++) { const g = Math.abs(gz(k)); gzSum += g; if (g > gzMax) gzMax = g; if (k < i + K.HOLD) { hSum += g; if (g > hMax) hMax = g; } }
    if (gzMax < inRange) { const w = gzSum / (e - i) * D2R; if (Math.hypot(mx, my) > w * w * radius / GMS * 1.5 + 1.5) return REL.FIT; }
    else if (hMax < inRange) { const [hx, hy] = inMed(i, i + K.HOLD), w = hSum / K.HOLD * D2R; if (Math.hypot(hx, hy) > w * w * radius / GMS * 1.5 + 1.5) return REL.FIT; }
    const ge = Math.min(i + K.HOLD + K.GAP, end);
    for (let k = i + K.HOLD; k < ge; k++) if (comp(k, mx, my, mz) > gapLim) return REL.GAP;
    if (L.relMinRpm > 0) {
      const m = Math.min(i + K.STEADY_N, end); let sum = 0, lo = 1e9, hi = 0, pos = 0, neg = 0;
      for (let k = i; k < m; k++) { const g = gz(k), a = Math.abs(g); sum += a; if (a < lo) lo = a; if (a > hi) hi = a; if (g > 0) pos++; else neg++; }
      const mean = sum / (m - i);
      if (mean < L.relMinRpm * 6 || (pos && neg) || hi - lo > 0.3 * mean + 60) return REL.SPIN;
    }
    return REL.OK;
  };
  // the analysis' inputs, as the disc's info packet would give them
  const infoFor = () => ({ odr: K.ODR, alsb: AL, glsb: GL, bias: bias.slice(), radius, logVer: ver, sens: cfg.level || 5,
    freeG: L.freeG, trigG: L.trigG, minPeak: L.minPeakG, minSpin: L.minSpinRpm, relMinRpm: L.relMinRpm, relPeakG: L.relPeakG,
    prePeak: K.PRE_PEAK, steadyN: K.STEADY_N, biasTol: K.BIAS_TOL, calFlags: (cfg.locked && radius > 0 ? 2 : 0) | 64,
    rateHz: cfg.rateHz || 0 });

  // ---- the loop's state (names as in disc_tracker_v2.ino)
  let preFill = 0, capturing = false, cap = null, refrEnd = 0, tail = null, rearmHold = false, rearmStillN = 0, rearmSeq = 0;

  const finalize = (k) => {
    const c = cap, I = infoFor();
    let R = null;
    try { R = an.analyzeCapture(S.subarray(c.start * 6, (c.start + c.n) * 6), I); } catch (err) { R = { error: String(err), flags: 0, n: c.n }; }
    c.R = R; c.info = I;
    const FL = an.FL;
    // VALID from analyze()'s own rule. sim.html's copy also applies finalizeThrow's hard-pull rule (LOW_CONF); the
    // disc applies it here, with the live peak, so it is worked out again below as the firmware does.
    let valid = !!(R.flags & FL.VALID) && !(R.flags & FL.LOW_CONF), why = WHY.COUNTED, lowConf = false;
    if (!valid) {
      why = R.peak <= L.minPeakG ? WHY.PEAK : R.spin < L.minSpinRpm ? WHY.SPIN : WHY.NO_RELEASE;
      const peak = Math.max(R.peak || 0, c.peakLive);
      let spinN = 0, gzMax = 0;
      if (peak >= K.HARD_PULL_G)
        for (let i = c.start; i < c.start + c.n; i++) { const g = Math.abs(gz(i) - bias[2]); if (g >= K.HARD_SPIN_DPS) spinN++; if (g > gzMax) gzMax = g; }
      if (peak >= K.HARD_PULL_G && spinN >= K.HARD_SPIN_N) { valid = true; lowConf = true; c.spinFloor = gzMax / 6; }
    }
    c.counted = valid; c.why = valid ? WHY.COUNTED : why; c.lowConf = lowConf;
    if (!valid) {
      c.quiet = c.end === END.LIMIT && R.spin < 0.5 * L.relMinRpm && c.peakLive < K.QUIET_MAX_G;
      if (c.quiet) c.why = WHY.HANDLING;
      refrEnd = k + 1;                                        // re-armed at once: the real throw may come next
      return;
    }
    refrEnd = k + 1 + refrN;
    if (v6) { rearmHold = true; rearmStillN = 0; rearmSeq = k; }
    // the spin radius learned from a throw the gyro measured (not when the calibration page locked it)
    if (!cfg.locked && R.rLearn > 0.0005 && R.rLearn < 0.03) { radius = radius > 0 ? 0.7 * radius + 0.3 * R.rLearn : R.rLearn; c.radiusAfter = radius; }
    if (c.end === END.WINDOW) {                               // still in the air: record the rest of the flight
      tail = { start: k + 1, count: 0, landAt: 0, hist: [], wait: null, link: c };
      c.tail = { start: k + 1, end: -1, landAt: -1, why: "" };
    }
  };
  const endTail = (k, why) => {
    const t = tail.link.tail;
    t.end = k; t.why = why; t.landAt = tail.landAt ? tail.start + tail.landAt - 1 : -1;
    tail = null;
    refrEnd = k + 1 + refrN;
    rearmSeq = k; rearmStillN = 0;
  };
  // tailFeed(): the landing. v6+: a reading LAND_G from the middle of the readings before it (up to LAND_HIST of them),
  // with a jump, and the spin then dropping, or LAND_BIG_G on its own. v5: LAND_G from the median of the last 12.
  const tailFeed = k => {
    const t = tail; t.count++;
    const a = [ax(k), ay(k), az(k)], gsat = sat(S[k * 6 + 2]), g = Math.abs(gz(k) - bias[2]);
    if (!v6) {
      if (!t.landAt && t.hist.length === 12) {
        let d2 = 0; for (let j = 0; j < 3; j++) { const d = a[j] - medianOf(t.hist.map(h => h.a[j])); d2 += d * d; }
        landDev[k] = Math.sqrt(d2);
        if (d2 > K.LAND_G * K.LAND_G) t.landAt = t.count;
      }
      t.hist.push({ a }); if (t.hist.length > 12) t.hist.shift();
    } else {
      if (!t.landAt && t.wait) {
        const low = t.wait.sat ? !gsat : g < 0.85 * t.wait.level;
        t.wait.run = low ? t.wait.run + 1 : 0;
        if (t.wait.run >= 3) t.landAt = t.wait.at;
        else if (t.count > t.wait.at + K.LAND_CONFIRM) t.wait = null;
      } else if (!t.landAt && t.hist.length >= 12) {
        let jump = 0, p = a;
        for (let q = 1; q <= 3 && q <= t.hist.length; q++) { const h = t.hist[t.hist.length - q].a; jump = Math.max(jump, Math.hypot(p[0] - h[0], p[1] - h[1], p[2] - h[2])); p = h; }
        if (jump >= K.LAND_JUMP_G) {
          const m = [0, 1, 2].map(j => medianOf(t.hist.map(h => h.a[j])));
          const d = Math.max(Math.hypot(a[0] - m[0], a[1] - m[1]), Math.abs(a[2] - m[2]));
          landDev[k] = d;
          if (d > K.LAND_BIG_G && jump >= K.LAND_BIG_JUMP) t.landAt = t.count;
          else if (d > K.LAND_G) {
            const allSat = t.hist.every(h => h.sat), level = medianOf(t.hist.map(h => h.g));
            if (allSat || level > 120) t.wait = { sat: allSat, level, at: t.count, run: 0 };
            else t.landAt = t.count;
          }
        }
      }
      t.hist.push({ a, g, sat: gsat }); if (t.hist.length > K.LAND_HIST) t.hist.shift();
    }
    if (t.landAt && t.count >= t.landAt + K.TAIL_AFTER) endTail(k, "landed");
    else if (t.count >= K.TAIL_MAX) endTail(k, "10 s limit");
  };

  for (let k = 0; k < n; k++) {
    if (cfg.gap && cfg.gap[k]) {                              // samples lost before this one (FIFO overrun, a stream gap)
      if (capturing) { cap.end = END.LOST; cap.endK = k - 1; cap.counted = false; cap.why = WHY.LOST; captures.push(cap); capturing = false; refrEnd = k; }
      if (tail) endTail(k - 1, "samples lost");
      preFill = 0;
    }
    if (!capturing) {
      if (preFill < K.PRE) preFill++;
      if (tail) { state[k] = STATE.TAIL; tailFeed(k); continue; }
      if (k < refrEnd) { state[k] = STATE.REFRACTORY; continue; }
      if (preFill < K.PRE) { state[k] = STATE.FILLING; continue; }
      const aM = mag(k), gM = rot(k);
      if (rearmHold) {
        rearmStillN = gM < K.REARM_DPS && Math.abs(aM - 1) < K.REARM_G ? rearmStillN + 1 : 0;
        const timeUp = k - rearmSeq > rearmMaxN && gM < L.trigDps;
        if (rearmStillN >= K.REARM_STILL_N || timeUp) rearmHold = false;
        else { state[k] = STATE.SETTLING; continue; }
      }
      if (aM < L.trigG && gM < L.trigDps) { state[k] = STATE.ARMED; continue; }
      // the trigger: the capture is the last PRE samples, this one last
      const start = k - (K.PRE - 1);
      let peakLive = 0;
      for (let i = start + (K.PRE_PEAK < K.PRE - 1 ? K.PRE - 1 - K.PRE_PEAK : 0); i < k; i++) peakLive = Math.max(peakLive, mag(i));
      cap = { trig: k, trigBy: aM >= L.trigG ? (gM >= L.trigDps ? "both" : "force") : "rotation", trigForce: aM, trigRot: gM,
              start, n: K.PRE, cand: k, rel: -1, peakLive, dcN: 0, dc: [0, 0, 0], level: 0, hitAt: 0, hitRun: 0, end: 0, endK: -1, hitK: -1,
              radiusBefore: radius };
      capturing = true; state[k] = STATE.CAPTURE;
      continue;
    }
    // ---- capturing
    const c = cap; c.n++;
    const capEnd = c.start + c.n;                             // stream index one past the newest sample
    let why = 0;
    if (c.rel < 0) {
      state[k] = STATE.CAPTURE;
      while (c.cand + K.WIN <= capEnd) {
        const m = mag(c.cand); if (m > c.peakLive) c.peakLive = m;
        const code = c.peakLive > L.relPeakG ? releaseAt(c.cand, capEnd, c.peakLive) : REL.PEAK;
        relCode[c.cand] = code;
        if (code === REL.OK) { c.rel = c.cand; c.relFoundAt = k; break; }
        c.cand++;
      }
    } else {
      state[k] = STATE.FLIGHT;
      const since = capEnd - c.rel, aM = mag(k);
      if (since > K.SETTLE && c.dcN < K.WIN) {
        const a = c.rel + K.SETTLE, [mx, my] = inMed(a, capEnd);
        c.dc = [mx, my, v6 ? zMed(a, capEnd) : 0];
        if (v6) c.level = spinLevel(a, capEnd);
        c.dcN = capEnd - a;
      }
      const aImp = c.dcN >= 12 ? comp(k, c.dc[0], c.dc[1], c.dc[2]) : aM;
      if (c.dcN >= 12) impForce[k] = aImp;
      if (v6) {
        const hit = since > K.SETTLE && c.dcN >= 12 && aImp > K.IMPACT_G;
        let hitEnd = 0;
        if (c.hitAt) {
          const g = Math.abs(gz(k)), low = c.level >= inRange ? g < inRange : g < 0.85 * c.level;
          c.hitRun = low ? c.hitRun + 1 : 0;
          if (c.hitRun >= 3) hitEnd = c.hitAt;
          else if (c.n > c.hitAt + K.LAND_CONFIRM) c.hitAt = 0;
        } else if (hit) {
          if (aImp > K.IMPACT_BIG_G || c.level < 120) hitEnd = c.n - 1;
          else { c.hitAt = c.n - 1; c.hitRun = 0; }
        }
        if (hitEnd) {
          const minKeep = c.rel - c.start + K.SETTLE + 1;
          c.hitK = c.start + hitEnd;
          c.n = hitEnd > minKeep + K.IMPACT_TRIM ? hitEnd - K.IMPACT_TRIM : minKeep;
          why = END.IMPACT;
        } else if (since >= K.SETTLE + K.FLIGHT) why = END.WINDOW;
      } else if (since > K.SETTLE && c.dcN >= 12 && aImp > K.IMPACT_G) {
        const flight = since - K.SETTLE;
        c.hitK = k;
        c.n -= 1 + (flight > K.IMPACT_TRIM ? K.IMPACT_TRIM : flight - 1);
        why = END.IMPACT;
      } else if (since >= K.SETTLE + K.FLIGHT) why = END.WINDOW;
    }
    if (!why && c.n >= K.MAXS) why = END.LIMIT;
    if (why) {
      c.end = why; c.endK = k; c.nUntrimmed = capEnd - c.start;
      capturing = false;
      finalize(k);
      captures.push(c);
      preFill = K.PRE;                                        // the ring is refilled from the capture's last samples
    }
  }
  if (capturing) { cap.end = 0; cap.endK = n - 1; cap.unfinished = true; captures.push(cap); }
  if (tail) { tail.link.tail.end = n - 1; tail.link.tail.why = "recording ended"; tail.link.tail.landAt = tail.landAt ? tail.start + tail.landAt - 1 : -1; }
  return { captures, state, relCode, impForce, landDev, cfg: Object.assign({}, cfg, { radiusEnd: radius }), level: L, K };
}

G.DiscFirmware = { run, SENS, K, STATE, REL, WHY, END };
})(typeof window !== "undefined" ? window : globalThis);
