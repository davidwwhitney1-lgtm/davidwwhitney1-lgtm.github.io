// simcode.js - the disc's own analysis, taken out of sim.html at run time, so the bench and lab.html use exactly what
// the pages run (rule R5: sim.html's analyzeCapture gives the decisions of the firmware's throw_metrics.h).
//
// sim.html keeps analyzeCapture inside its module script. Copying it here would make two versions that drift apart,
// so this file cuts the blocks it needs out of sim.html's text (as the author's R5 check does) and builds them into
// one function. Each block starts at a known line ("function analyzeCapture(S, I)") and ends at the first "}" on a
// line of its own.
//
// Browser: DiscSimCode.load() fetches sim.html from the same folder. Node: DiscSimCode.build(text, DiscTrack).
// Either way the result is { analyzeCapture, fillAccel, discAngles, relRules, FL, Q4, medianOf, REL_V4 }.
(function (G) {
"use strict";
function build(html, DiscTrack) {
  html = html.replace(/\r\n/g, "\n");
  const block = start => {
    const i = html.indexOf(start); if (i < 0) throw new Error(`sim.html has no "${start}"`);
    const j = html.indexOf("\n}\n", i); if (j < 0) throw new Error(`"${start}" in sim.html has no end`);
    return html.slice(i, j + 3);
  };
  const line = start => { const i = html.indexOf(start); if (i < 0) throw new Error(`sim.html has no "${start}"`); return html.slice(i, html.indexOf("\n", i) + 1); };
  const stmt = start => { const i = html.indexOf(start); if (i < 0) throw new Error(`sim.html has no "${start}"`); return html.slice(i, html.indexOf(";\n", i) + 2); };
  const obj = start => { const i = html.indexOf(start); if (i < 0) throw new Error(`sim.html has no "${start}"`); return html.slice(i, html.indexOf("};", i) + 2) + "\n"; };
  const code = [
    "const D2R = Math.PI / 180, R2D = 180 / Math.PI, GMS = 9.80665;",
    stmt("const REL_V4 = ["),
    line("const medianOf = arr"),
    block("class Q4 {"),
    block("function discAngles("),
    block("function relRules(I)"),
    obj("const FL = {"),
    block("function fillAccel("),
    block("function analyzeCapture(S, I)"),
    "return { analyzeCapture, fillAccel, discAngles, relRules, FL, Q4, medianOf, REL_V4 };",
  ].join("\n");
  // eslint-disable-next-line no-new-func
  return new Function("DiscTrack", code)(DiscTrack);
}
async function load(url = "sim.html") {
  const r = await fetch(url, { cache: "no-cache" });
  if (!r.ok) throw new Error(`could not read ${url} (${r.status})`);
  return build(await r.text(), G.DiscTrack);
}
G.DiscSimCode = { build, load };
})(typeof window !== "undefined" ? window : globalThis);
