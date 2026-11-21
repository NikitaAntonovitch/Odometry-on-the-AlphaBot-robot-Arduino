# D6 — TRSensors Library Explainer (annotated source)

All quotes are from `TRSensors/TRSensors/TRSensors.cpp` / `.h` in this project
(the Waveshare-modified Pololu QTRSensors derivative). Line numbers refer to the
files as of 2026-11-13. This is the document to re-read on the bus before the demo.

**2026-11-21 (teacher fix): the sketch no longer calls `readLine()`.** The
teacher requires sensor input through the two intended calls only:
`AnalogRead()` and `readCalibrated()`. Line position and lost-line detection
are OUR OWN code (see sections 4b/4c). Section 4 stays because the instructor
may still ask what the library's `readLine` does and why we were right NOT to
rely on its sentinel behaviour.

## 1. `AnalogRead(unsigned int *sensor_values)` — lines 49–84

```cpp
#define Clock     13
#define Address   12
#define DataOut   11
#define CS        10
...
for(j = 0;j < _numSensors + 1;j++)     // 6 passes for 5 sensors
{
    digitalWrite(CS,LOW);
    for(i = 0;i < 10;i++)
    {
        //0 to 4 clock transfer channel address
```

Why 6 passes for 5 sensors: the TLC1543 ADC returns the *previous* conversion
while the *next* channel address is being clocked in. The result of read n
arrives one transfer late, so one extra dummy pass (j = 5) flushes the pipeline;
without it sensor 0's value would be stale. The 10 clocks per pass carry the
channel address bits (0–4) plus the conversion.

In the new design `AnalogRead` is used by the `'r'` serial command only —
raw diagnostics, never line position.

## 2. `calibrate()` — lines 84–123

```cpp
for(j=0;j<10;j++)
{
    AnalogRead(sensor_values);
    for(i=0;i<_numSensors;i++)
    {
        if(j == 0 || max_sensor_values[i] < sensor_values[i])
            max_sensor_values[i] = sensor_values[i];
        if(j == 0 || min_sensor_values[i] > sensor_values[i])
            min_sensor_values[i] = sensor_values[i];
    }
}
// record the min and max calibration values
for(i=0;i<_numSensors;i++)
{
    if(min_sensor_values[i] > calibratedMax[i])
        calibratedMax[i] = min_sensor_values[i];
    if(max_sensor_values[i] < calibratedMin[i])
        calibratedMin[i] = max_sensor_values[i];
}
```

Key points (all [HW]-confirmed 2026-11-09):
- ONE `calibrate()` call does only 10 readings and updates the running extremes —
  it cannot see the full contrast by itself.
- We need ~400 calls (~10 s) WITH MOVEMENT so every sensor sees both the line
  and the background. In our sketch `W` runs `runCalibration()` for that purpose.
- Constructor initialises `calibratedMin[i]=1023, calibratedMax[i]=0` (lines 37–41):
  the "identity" extremes that make an uncalibrated `readCalibrated` clamp to 0.
- **NEW (teacher requirement 4): after every calibration the sketch runs
  `measurePolarityFromCalibration()`, which decides the line polarity FROM THIS
  DATA** — `mean(calibratedMin)` vs `mean(calibratedMax)` across the 5 sensors:
  the dark end of the ranges is what the line looks like, the bright end what
  the background looks like. Dark line → `lineDarkFlag=true` → every calibrated
  value is inverted (1000 − v) before averaging. Nothing is hard-coded; the
  same measurement works on a bright line over dark floor.

## 3. `readCalibrated(unsigned int *sensor_values)` — lines 126–156

```cpp
denominator = calibratedMax[i] - calibratedMin[i];
signed int x = 0;
if(denominator != 0)
    x = (((signed long)sensor_values[i]) - calibratedMin[i])
        * 1000 / denominator;
if(x < 0) x = 0;
else if(x > 1000) x = 1000;
sensor_values[i] = x;
```

Exact rescale: `(raw - min) * 1000 / (max - min)`, clamped to 0..1000, per
sensor. 0 = the brightest surface seen during calibration, 1000 = the darkest.
The `denominator != 0` guard is why stationary calibration is fatal in
practice: min ≈ max for every sensor → x ≈ 0 everywhere → no contrast at all
(our [HW] observation with the old path: ranges 22/526/14/32/407, position pinned).

**This is now the ONLY data source for line position** — exactly the teacher's
requirement 1b.

## 4. `readLine(unsigned int *sensor_values, unsigned char white_line)` — lines 178–223 (NOT USED any more — kept for the demo question)

```cpp
static int last_value=0; // assume initially that the line is left.
...
if(value > 300) { on_line = 1; }            // line-under-sensor threshold
if(value > 50) {                            // noise gate for the average
    avg += (long)(value) * (i * 1000);
    sum += value;
}
...
if(!on_line)
{
    // If it last read to the left of center, return -1.
    if(last_value < (_numSensors-1)*1000/2)
        return -1;
    // If it last read to the right of center, return the max+1.
    else
        return (_numSensors-1)*1000+1;
}
last_value = avg/sum;
return last_value;
```

What the instructor may ask, and the honest answer:
- The library's `readLine` computes the SAME weighted average we now compute
  ourselves, but with two drawbacks the teacher flagged: it inverts values
  internally (`value = 1000 - value` when `white_line=0`) with a header comment
  that claims the opposite (doc-vs-code bug — good demo answer), and it reports
  "line lost" with the −1/4001 sentinels, which requires signed-int handling
  everywhere and cannot be unit-tested as a pure function.
- Our replacement (`computePosition()`, section 4b) has no sentinels: it returns
  a position AND a separate `lineVisible` flag. Cleaner to test (host tests
  9–15 in `pid_test.cpp` cover it) and impossible to mix up with valid 0..4000
  positions.
- The unsigned-int trap is still worth mentioning: −1 stored in `unsigned int`
  becomes 65535; the sign is destroyed. Our position math avoids the problem
  completely because no sentinel is ever returned.

## 4b. OUR line position: `computePosition()` (sketch, Stage 2)

```cpp
long computePosition(bool &lineVisible) {
  trs.readCalibrated(sensorValues);        // teacher requirement 1b
  long sum = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int v = (int)sensorValues[i];
    if (lineDarkFlag) v = 1000 - v;        // polarity correction (measurement-decided)
    if (v < 0) v = 0;
    sensorValues[i] = (unsigned int)v;
    sum += v;
  }
  if (sum < lineLostSumMin) {              // teacher requirement 5: lost = low SUM
    lineVisible = false;
    return -1;                             // "no data" marker, never used as position
  }
  long weighted = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    weighted += (long)sensorValues[i] * ((long)i * 1000);
  lineVisible = true;
  return weighted / sum;                   // 0..4000, 2000 = centred
}
```

- Position is ONE number — the weighted average over all 5 corrected values
  (teacher requirement 3): `(0*v0 + 1000*v1 + 2000*v2 + 3000*v3 + 4000*v4) / (v0+..+v4)`.
  `error = position − 2000`, so error 0 = line centred.
- Lost line = SUM test (requirement 5): after polarity correction the line is
  always the HIGH side, so `sum` of the 5 values is high when any sensor is on
  the line and near zero when none is. No fake position is returned in the
  lost case — the caller gets `lineVisible=false`.
- `readError()` wraps it: on a lost tick it returns the LAST VALID error
  (`lastGoodError`, updated only on visible readings) so the search direction
  follows the last valid error sign (requirement 6). `lineLost()` now reports
  the state flag, not an error range.

## 4c. Why the sum works (demo soundbite)

With polarity corrected, EVERY sensor reading means the same thing:
value ≈ how much of that sensor sits on the line. Background → ~0
(realistically 20–50 residuals), line → hundreds. So:
- sum < lineLostSumMin (default 250, NEEDS HW TUNING) → no sensor on the line.
- several sensors high at once (crossing stripe) → crossing detector
  counts sensors ≥ 300 corrected; behaviour unchanged.

## 5. Pin assignments — why the sketch never touches D10–D13

`Clock=D13, Address=D12, DataOut=D11, CS=D10` are `pinMode`-ed and driven inside
the constructor (lines 25–31). The sketch only calls `AnalogRead/readCalibrated/
calibrate`; touching those pins directly would corrupt the TLC1543
transfer. (This is also why the status LED lives on D8 in our sketch, not D13.)

## 6. Sensor index trap

The board silkscreen says IR1–IR5; the library (and every trace line) uses
`sensorValues[0..4]`. IR1 = index 0. Our calibratedMin/Max dumps and the `r`/`c`
commands print indices 0..4. [HW]: index 0 currently has the weakest MIN/MAX range.

## 7. No hardware claims

Everything above is source-quoted [CODE]; only the marked items are [HW]
(2026-11-09 school session, old readLine-based path). The NEW computePosition
path (cal-based position, sum lost-line, measured polarity) is host-tested
15/15 [SIM] but NOT yet verified on the robot — see hardware_checklist_2026-11-21.md.
