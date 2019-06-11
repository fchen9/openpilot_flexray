### FlexRay timing params
- gdCycle, full cycle time, in us.
- gMacroPerCycle, full cycle time, in macroticks.
- gdStaticSlot, length of static slot, in macroticks.
- gNumberOfStaticSlots
- gdMinislot, length of dynamic slot, in macroticks.
- gNumberOfMinislots
- gdNIT, the network idle time.
- gdActionPointOffset
- gdMinislotActionPointOffset
- gdSymbolWindow, the length og symbol window, for CAS transmission
- gOffsetCorrectionMax, the maximum amount of offset correction.
- pOffsetCorrectionOut, the upper bound for a permissible offset
correction.
- pRateCorrectionOut, the upper bound for a permissible rate correction.

### FlexRay params constraints
- Constraint 18: 
```
gMacroPerCycle[MT] = gdStaticSlot[MT] * gNumberOfStaticSlots + adActionPointDifference[MT] +
gdMinislot[MT] * gNumberOfMinislots + gdSymbolWindow[MT] + gdNIT[MT]
```
  - *gdStaticSlot[MT] * gNumberOfStaticSlots + adActionPointDifference[MT]* is the total length of static segment.
  - *gdMinislot[MT] * gNumberOfMinislots* is the total length of dynamic segment.

- Equation 12: 
```
adActionPointDifference[MT] = 0 if gdActionPointOffset <= gdMinislotActionPointOffset
= 0 if gNumberOfMinislots = 0
= gdActionPointOffset - gdMinislotActionPointOffset else
```

- Equation 17: 
```
[17] adOffsetCorrection[MT] >= ceil( gOffsetCorrectionMax[µs] / (gdMacrotick[µs/MT] *
(1 - cClockDeviationMax) - gdMaxMicrotick[µs/µT] *
cMicroPerMacroMin[µT/MT] * (1 + cClockDeviationMax)) )
```

- Constraint 26: 
```
gOffsetCorrectionStart[MT] = gMacroPerCycle[MT] - adOffsetCorrection[MT]
```
adOffsetCorrection is the length of the offset correction phase.

### FlexRay timing params value range
- gdSymbolWindow is in range [0, 162]
- gdMiniSlotActionPointOffset is in range [1, 31]
- gdActionPointOffset is in range [1, 63]
- gdNIT is in range [2, 15978]
- gNumberOfMinislots is in range [0, 7988]
- gdMiniSlot is in range [2, 63]

### What we have for AUDI A4, from waveform decoding using pico scope.
- gdCycle = 5000us
- gMacroPerCycle = 5000 MT
- gdStaticSlot = 63
- gNumberOfStaticSlots = 51
- gdSymbolWindow, seems zero, because we can't find CAS from the waveform, not sure about this because the waveform is incompleted due to scope limitation.

### A design of Brute-force algorithm for timing params estimation
- Intialize gdNIT as 2, the min value of gdNIT.
- Set gdSymbolWindow to zero.
- Estimate *gdMinislot[MT] * gNumberOfMinislots* by doing calculation according to constraint 18 and Equation 12, and calculate the factors to find a valid pair of gdMinislot and gNumberOfMinislots values.
 - For example, for gdMinislot[MT] * gNumberOfMinislots = 1000, we set gdMinislot to 2 and gNumberOfMinislots to 500.
- Increase gdNIT by 1 for every recursion

### TODO
- Brute-force gOffsetCorrectionStart.
- Brute-force gOffsetCorrectionMax.
- Brute-force pOffsetCorrectionOut.
- Brute-force pRateCorrectionOut.
