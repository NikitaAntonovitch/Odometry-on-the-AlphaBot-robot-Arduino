# D6 — TRSensors Library Explainer (annotated source)

All quotes are from `TRSensors/TRSensors/TRSensors.cpp` / `.h` in this project
(the Waveshare-modified Pololu QTRSensors derivative). Line numbers refer to the
files as of 2026-11-13. This is the document to re-read on the bus before the demo.

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
sensor. The `denominator != 0` guard is why stationary calibration is fatal in
practice: min ≈ max for every sensor → x ≈ 0 everywhere → `readLine` sees no
line at all (our [HW] observation: linePos pinned at 2000 with ranges 22/526/14/32/407).

## 4. `readLine(unsigned int *sensor_values, unsigned char white_line)` — lines 178–223

```cpp
static int last_value=0; // assume initially that the line is left.
...
for(i=0;i<_numSensors;i++) {
    int value = sensor_values[i];
    if(!white_line)
        value = 1000-value;          // dark line: invert
    sensor_values[i] = value;
    if(value > 300) { on_line = 1; }            // line-under-sensor threshold
    if(value > 50) {                            // noise gate for the average
        avg += (long)(value) * (i * 1000);
        sum += value;
    }
}
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

- Weighted average: `(0*v0 + 1000*v1 + 2000*v2 + 3000*v3 + 4000*v4) / (v0+..+v4)`.
- `white_line=0` (our choice, [HW]): line is dark → values inverted before
  averaging (`value = 1000-value`); 300 = on-line threshold, 50 = noise gate.
- **The lost-line modification (this is the Waveshare change vs Pololu original):**
  `if(!on_line) return -1;` / `return (_numSensors-1)*1000+1;` i.e. **−1 or 4001**.
  The manufacturer's original returned 0 or 4000 — indistinguishable from "line
  exactly under sensor 0 / sensor 4", which makes lost-line detection impossible
  without extra state. With −1/4001 the sketch's `readError()` (linePos − 2000)
  yields −2001 / +2001, both outside ±2000 → `lineLost()` true, and the SIGN of
  the error tells the search direction (`lastSeenSide = (e<0) ? -1 : +1`).
- Demo risk to mention: −1 survives only in a SIGNED variable. The assignment's
  unsigned-int trap: −1 stored in `unsigned int` becomes 65535; after subtracting
  2000 → 63535; `|error| > 2000` still trips but the sign is destroyed and the
  robot would search the wrong way. Our `readError()` returns signed `int` — verified.

## 5. Pin assignments — why the sketch never touches D10–D13

`Clock=D13, Address=D12, DataOut=D11, CS=D10` are `pinMode`-ed and driven inside
the constructor (lines 25–31). The sketch only calls `AnalogRead/readCalibrated/
readLine/calibrate`; touching those pins directly would corrupt the TLC1543
transfer. (This is also why the status LED lives on D8 in our sketch, not D13.)

## 6. Sensor index trap

The board silkscreen says IR1–IR5; the library (and every trace line) uses
`sensorValues[0..4]`. IR1 = index 0. Our calibratedMin/Max dumps and the `r`/`c`
commands print indices 0..4. [HW]: index 0 currently has the weakest MIN/MAX range.

## 7. No hardware claims

Everything above is source-quoted [CODE]; only the marked items are [HW]
(2026-11-09 school session). Behaviour of `readLine` during a fast spin
(1–2-tick sweep vs 3-consecutive-readings requirement) is [UNVERIFIED on hardware].
