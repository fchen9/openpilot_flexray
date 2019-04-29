![](https://i.ibb.co/wQyCbN8/symbolwindow-error.png)

#### POC:integration consistency
FlexRay spec 2.1 page 163: As soon as a valid startup frame has been received in one of the listen states (see Figure 7-8), the
POC:initialize schedule state is entered. If clock synchronization successfully receives a matching second
valid startup frame and derives a schedule from them (indicated by receiving the signal integration
successful on A or integration successful on B), the POC:integration consistency check state is entered.
Otherwise the node reenters the appropriate listen state.

#### Symbol Window Syntax Error on Channel A
MPC5748GRM page 2105 : Symbol Window Syntax Error on Channel A. protocol related variable: vSS!SyntaxError for symbol window
on channel A. This status bit is set when a syntax error was detected during the symbol window on channel A.

#### Media Access Test Symbol MTS Received on Channel A
MPC5748GRM page 2105 : Media Access Test Symbol MTS Received on Channel A. protocol related variable: vSS!ValidMTS for
symbol window on channel A. This status bit is set if the Media Access Test Symbol MTS was received in the symbol window on channel
A.

#### What is MTS symbol
 - LOW bit transmited during symbol window segment, after TSS.
 - FlexRay spec 2.1 page 67: Collision avoidance symbol and media access test symbol decoding
The node shall decode the CAS and MTS symbols in exactly the same manner. Since these symbols are
encoded by a LOW level of duration cdCAS starting immediately after the TSS, it is not possible for receivers
to detect the boundary between the TSS and the subsequent LOW bits that make up the CAS or MTS.
As a result, the detection of a CAS or MTS shall be considered as valid coding if a LOW level with a duration
between cdCASRxLowMin and gdCASRxLowMax is detected.

#### vSS!SyntaxError and vSS!ValidMTS
![](https://i.ibb.co/cgw658r/v-SS-Syntax-Error.png)


#### Summary
 - The error occured at the last stage of startup: "POC:integration consistency check", after startup frames pairs have been received successfully. 
 - Clock correction failed because the FlexRay CC received more than one MTS during symbol window segment.
 - Both TSS and MTS are LOW bits, so this error means the LOW bits received during symbol window is too long.

#### Timing Constraints
```bash 
Constraint 18:
gMacroPerCycle[MT] = gdStaticSlot[MT] * gNumberOfStaticSlots + adActionPointDifference[MT] + gdMinislot[MT] * gNumberOfMinislots + gdSymbolWindow[MT] + gdNIT[MT]
```
```bash
Constraint 16:
gdSymbolWindow[MT] = 2 * gdActionPointOffset[MT] + ceil( ((gdTSSTransmitter[gdBit] + cdCAS[gdBit] +
cChannelIdleDelimiter[gdBit]) * gdBitMax[µs/gdBit] + dBDRxai[µs] +
gdMinPropagationDelay[µs] + gdMaxPropagationDelay[µs]) /
(gdMacrotick[µs/MT] * (1 - cClockDeviationMax)) )
```
