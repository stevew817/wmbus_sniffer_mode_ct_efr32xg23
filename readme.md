# Customised version of EFR32 sample application "RAIL - SoC Wireless M-bus Meter and Collector"

This implementation receives frames on Wireless M-Bus modes C (frame types A and B) and T simulataneously (on the M2O profile). Refer to EN 13757-4 for the details of these transmission modes. The code/concept is superficially tested on EFR32xG14 and EFR32xG23. The radio and hardware configurations in this repo are for EFR32xG23 SoC radio boards.

## Reference for the sample application
Wireless M-bus applications are documented in [AN1119: Using RAIL for Wireless M-Bus
Applications with EFR32](https://www.silabs.com/documents/public/application-notes/an1119-rail-wmbus.pdf)

## Modifications to the sample application

- Changed radio settings to match the common frame format between modes C and T
  - preamble to detect is set to 16 times '01' bit pattern (total preamble length 32 bits)
  - sync word to detect is set to 16 bits with value 0x543D
  - encoding is set to NRZ, i.e. raw bits
  - allowed deviation and tolerances are set empirically
  - channel settings are set according to EN13757-4
  - packet length is configured as constant (but is changed dynamically for each packet as it comes in through a callback function from the radio)
- Enabled event generation for 'RX Sync1 Detect' in 'RAIL Utility, initialisation of inst0' configuration in order to trigger the per-packet packet length detection in software
- Implemented per-packet mode and packet length detection in software
  - This is done by setting a timer to expire 260us (26 bit times) after the Sync1 detect event. This gives the radio time to have received at least 24 bits at the time when the timer callback is called.
  - Inside the timer callback, the goal is to set the correct packet length for the current transmission. To that end, we check first which mode we think the frame is in:
    - If the first received byte is 0x54 ('01010100') then we assume it is mode C, since the pattern '010101' is not a valid 3of6 pattern.
      - If the second received byte is 0xCD ('11001101') then the packet is deemed mode C frame type A, since 0x54CD is the preamble for such a packet. That means the third byte is the L-field, and the packet length is calculated according to interpreting the L-field as part of frame A.
      - If the second received byte is 0x3D ('00111101') then the packet is deemed mode C frame type B, since 0x543D is the preamble for such a packet. That means the third byte is the L-field, and the packet length is calculated according to interpreting the L-field as part of frame B.
    - If the first received byte is not 0x54, check whether the first 12 bits can be successfully decoded through the 3of6 decoding mechanism.
      - If they are valid 3of6 encodings, the first 12 bits received will decode in the first 8 bits of the packet, which is the L-field of a frame of type A. We now use this value to figure out the on-air packet length to set the radio to for the current packet.
- Implemented packet decoding in software
  - On reception of the full packet, we now have to:
    - Strip the sync bytes for a mode C packet
    - Decode the packet according to 3of6 decoding for a mode T packet
    - Manually check & strip CRC bytes in the packet (different for frame type A vs B)
- Added a raw packet print routine
  - Prints 'RX:{reception time}:{RSSI in dBm}:{mode ('T' or 'C')}:{frame ('A' or 'B')}:{Raw frame bytes in hex (CRCs not stripped)}\n'
- Added more verbose error printing
- Added AppHeader printing for long application headers in addition to short ones
