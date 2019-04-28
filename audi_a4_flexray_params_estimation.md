### Flexray cycle parameters
 - gdCycle: Full cycle time in µs.
 - gMacroPerCycle: The number of macroticks per cycle on the cluster.
 - pMicroPerCycle: The number of microticks per cycle on the node.

### Flexray timing parameters constraints (defined in flexray spec 2.1)
```python
Constraint 17:
gdMacrotick[µs] = gdCycle[µs] / gMacroPerCycle
Constraint 18:
gMacroPerCycle[MT] = gdStaticSlot[MT] * gNumberOfStaticSlots + adActionPointDifference[MT] +
gdMinislot[MT] * gNumberOfMinislots + gdSymbolWindow[MT] + gdNIT[MT]
Constraint 19:
pMicroPerCycle[µT] = round( gdCycle[µs] / pdMicrotick[µs/µT] )
```

### Cycle parameters estimated from picoscope waveform
- Suppose the flexray frame id is denoted by F, the cycle count is noted by X, the full cycle time can be estimated by:
 - gdCycle = （Start time of static frame id F in cycle count X + 1) - (Start time of static frame id F in cycle count X)
 - According to 20190427-0001_channel-A-B_10ms-div_250MS-s_Trigger-CHB-Rising-3,3V.psdata:
  - ![](https://i.ibb.co/jb22YQR/full-cycle.png)
```python  
F = 28
X = 43
gdCycle ~= (6.357ms - 1.3575ms) = 5ms 5000µs
Because pdMicrotick = 0.025µs, so:
pMicroPerCycle ~= 200000
```

### Frame params decoded from picoscope waveform
FlexRay parameters decoded from pin3.psdata:
```python
gNumberOfStaticSlots: 51
gPayloadLengthStatic: 17
gdBit 0.1 µs
pdMicrotick 0.025 µs
```python

Parameters estimated from 20181220-0001 rs pin 3.psdata waveform:
```python
gdTSSTransmitter: 9
```

Time interval between last static slot and first dynamic slot 52( gdActionPointOffset or gdMinislotActionPointOffset)
9 µs / gdMacrotick

Time interval between last static slot 51 and dynamic slot 54,  and no transmission on slot 52, 53
27 µs = gdActionPointOffset or gdMinislotActionPointOffset + 2 * gdMinislot

Time interval between dynamic slot 53 and dynamic slot 56,  and no transmission on slot 54, 55
26.8 µs, DTS LOW of slot 53 is 600ns

Time interval between static slots (related to gdActionPointOffset)
9 µs / gdMacrotick

Length of static slot waveform: 43.2 µs
aFrameLengthStatic 432 gdBit

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

Time interval between dynamic slots (related to gdMinislotActionPointOffset)
9 µs / gdMacrotick

dynamic trailing sequence:
slot id 52, payloadlength 4, period of LOW: 600ns
slot id 64, payloadlength 9, period of LOW: 200ns
slot id 65, payloadlength 17, period of LOW: 1600ns
slot id 64, payloadlength 9, period of LOW: 8600ns


gdSymbolWindow + gdNIT > 20µs / gdMacrotick
