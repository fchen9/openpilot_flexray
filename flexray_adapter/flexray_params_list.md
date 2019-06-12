### The goal of OpenPilot FlexRay project
Currently [OpenPilot](https://github.com/commaai/openpilot) can't support flexray based cars, like AUDI, BMW, Mercedes. Our goal is to fix this issues, run OpenPilot on these cars.
### How to add OpenPilot to a flexray car
- Make a flexray adapter for EON.
- Find Flexray parameters of the car, so that the flexray adapter can join into the flexray cluster.
- Use the flexray adapter to sniff on the car's flexray bus, decode steering/gas packets.
- Replace the LKAS camera with [EON](https://comma.ai/shop/products/eon-gold-dashcam-devkit) , let EON do lateral/longitudinal control.
- Done.
### Current Status
- We have made a flexray adapter, based on NXP MPC5748G devkit).
- We have partial of flexray params for AUDI A4, not enough for joining into the car's flexray network.
### Required flexray parameters
```
gColdStartAttempts
gListenNoise
gMaxWithoutClockCorrectionFatal
gMaxWithoutClockCorrectionPassive
gNetworkManagementVectorLength
gNumberOfMinislots
gNumberOfStaticSlots
gOffsetCorrectionMax
gOffsetCorrectionStart
gPayloadLengthStatic
gSyncFrameIDCountMax
gdActionPointOffset
gdCasRxLowMax
gdDynamicSlotIdlePhase
gdMacrotick
gdMaxInitializationError
gdMinPropagationDelay
gdMiniSlotActionPointOffset
gdMinislot
gdNIT
gdStaticSlot
gdSymbolWindow
gdTSSTransmitter
gdWakeupRxWindow
gdWakeupSymbolRxIdle
gdWakeupSymbolRxLow
gdWakeupSymbolTxActive
gdWakeupSymbolTxIdle
pAllowHaltDueToClock
pAllowPassiveToActive
pChannels
pClusterDriftDamping
pDecodingCorrection
pDelayCompensationA
pDelayCompensationB
pKeySlotId
pKeySlotOnlyEnabled
pKeySlotUsedForStartup
pKeySlotUsedForSync
pLatestTx
pMacroInitialOffsetA
pMacroInitialOffsetB
pMicroInitialOffsetA
pMicroInitialOffsetB
pMicroPerCycle
pOffsetCorrectionOut
pPayloadLengthDynMax
pRateCorrectionOut
pWakeupChannel
pWakeupPattern
pdAcceptedStartupRange
pdListenTimeout
pdMaxDrift
pdStarDelay
```
We welcome any help.
