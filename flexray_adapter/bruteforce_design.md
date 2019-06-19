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
See bf.py GUI for other params' value range.

### Decode some params from waveform, ease the bruteforce process
1. Guess some params
- We assume that most cars are running flexray at 10M bps speed, so we already have some timing params:
  - gdBit = 0.1 µs
  - pdMicrotick = 0.025 µs

2. Capture a waveform with high sample rate( > 50MS/s)
- gdTSSTransmitter can be decoded by picoscope. So gdTSSTransmitter = 920ns / gdBit = 9 below.
![](https://i.ibb.co/frmYdRM/image.png)
- gNumberOfStaticSlots, gPayloadLengthStatic
  - Filter our invalid frame by set "Header CRC Pass" and "Frame CRC A Pass" to 1
  - Sort the frames by frame id
  - Inspect the payload length, find the gPayloadLengthStatic and gNumberOfStaticSlots
![](https://i.ibb.co/X59KQ4S/image.png)
- Estimate gdActionPointOffset or gdMinislotActionPointOffset
  - Inspect the interval between the last static slot and the first dynamic slot
- Estimate gdStaticSlot
```
aFrameLengthStatic(gdBit) = config['gdTSSTransmitter'] + cdFSS + 80 + config['gPayloadLengthStatic'] * 20 + cdFES
aFrameLengthStatic(gdBit) = 9 + 2 + 80 + 17 * 20 + 1 = 432

Suppose gdMacrotick is 1 µs/MT
gdStaticSlot = 2 * config['gdActionPointOffset'] + ceil(((aFrameLengthStatic + cChannelIdleDelimiter) * gdBitMax +
		config['gdMinPropagationDelay'] + gdMaxPropagationDelay) / (config['gdMacrotick'] * (1 - cClockDeviationMax)))
gdBitMax = gdBit * (1 + cClockDeviationMax) = 0.1 * (1 + 0.0015) = 0.10015

0.0 < config['gdMinPropagationDelay'] + gdMaxPropagationDelay < 0.5

gdStaticSlot_min = 2 * 9 + ceil(((432 + 11) * 0.10015 + 0) / (1 * 0.9985)) = 63
gdStaticSlot_max = 2 * 9 + ceil(((432 + 11) * 0.10015 + 0.5) / (1 * 0.9985)) = 63
So gdStaticSlot is 63
```
2. Capture waveform into on single bufer  as much as possible
- Estimate the cycle length by searching for same frame id in adjacent cycle counters
![](https://i.ibb.co/GMGgRJm/image.png)
### What we have for AUDI A4, from waveform capturing and decoding using pico scope.
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
