# Wiring Diagram

## Complete System Wiring

```
                                 ┌─────────────────────────────────────────┐
                                 │           RASPBERRY PI 4                │
                                 │                                         │
                                 │  3.3V ────┬─────────────────────────┐  │
                                 │           │                         │  │
                                 │  5V ──────┼────┬────┬────┬────┬───┐│  │
                                 │           │    │    │    │    │   ││  │
                                 │  GND ─────┼────┼────┼────┼────┼───┼┤  │
                                 │           │    │    │    │    │   ││  │
                                 │  GPIO 2 ──┼────┼────┼────┼────┼───┼┤──┼── SDA
                                 │  (SDA)    │    │    │    │    │   ││  │
                                 │           │    │    │    │    │   ││  │
                                 │  GPIO 3 ──┼────┼────┼────┼────┼───┼┤──┼── SCL
                                 │  (SCL)    │    │    │    │    │   ││  │
                                 │           │    │    │    │    │   ││  │
                                 │  GPIO 4 ──┼────┼────┼────┼────┼───┼┼──┼── 1-Wire
                                 │           │    │    │    │    │   ││  │
                                 │  GPIO 14 ─┼────┼────┼────┼────┼───┼┼──┼── GPS TX
                                 │  (TX)     │    │    │    │    │   ││  │
                                 │           │    │    │    │    │   ││  │
                                 │  GPIO 15 ─┼────┼────┼────┼────┼───┼┼──┼── GPS RX
                                 │  (RX)     │    │    │    │    │   ││  │
                                 └───────────┼────┼────┼────┼────┼───┼┼──┘
                                             │    │    │    │    │   ││
        ┌────────────────────────────────────┘    │    │    │    │   ││
        │                                         │    │    │    │   ││
   ┌────┴────┐                               ┌────┴────┴────┴────┴───┴┴─────┐
   │ 4.7kΩ   │                               │         I2C BUS              │
   │ Pull-up │                               │                              │
   └────┬────┘                               │    ┌─────────────────────┐   │
        │                                    │    │                     │   │
        └────────────────────────────────────┼────┤ MPU6050 @ 0x68     │   │
                                             │    │ (Body Accelerometer)│   │
                                             │    └─────────────────────┘   │
                                             │                              │
                                             │    ┌─────────────────────┐   │
                                             │    │ TCA9548A @ 0x70     │   │
                                             ├────┤ I2C Multiplexer     │   │
                                             │    │                     │   │
                                             │    │ Ch0 ── MPU6050 (FL) │   │
                                             │    │ Ch1 ── MPU6050 (FR) │   │
                                             │    │ Ch2 ── MPU6050 (RL) │   │
                                             │    │ Ch3 ── MPU6050 (RR) │   │
                                             │    └─────────────────────┘   │
                                             │                              │
                                             │    ┌─────────────────────┐   │
                                             ├────┤ ADS1115 @ 0x48      │   │
                                             │    │ Front-Left Pot      │   │
                                             │    │ A0 ── Potentiometer │   │
                                             │    └─────────────────────┘   │
                                             │                              │
                                             │    ┌─────────────────────┐   │
                                             ├────┤ ADS1115 @ 0x49      │   │
                                             │    │ Front-Right Pot     │   │
                                             │    │ A0 ── Potentiometer │   │
                                             │    └─────────────────────┘   │
                                             │                              │
                                             │    ┌─────────────────────┐   │
                                             ├────┤ ADS1115 @ 0x4A      │   │
                                             │    │ Rear-Left Pot       │   │
                                             │    │ A0 ── Potentiometer │   │
                                             │    └─────────────────────┘   │
                                             │                              │
                                             │    ┌─────────────────────┐   │
                                             └────┤ ADS1115 @ 0x4B      │   │
                                                  │ Rear-Right Pot      │   │
                                                  │ A0 ── Potentiometer │   │
                                                  └─────────────────────┘   │
                                                                            │
                                             └──────────────────────────────┘


                    1-WIRE BUS (GPIO 4)
                    ══════════════════
                           │
                    ┌──────┴──────┐
                    │   4.7kΩ     │
                    │   Pull-up   │
                    │   to 3.3V   │
                    └──────┬──────┘
                           │
           ┌───────────────┼───────────────┬───────────────┬───────────────┐
           │               │               │               │               │
      ┌────┴────┐     ┌────┴────┐     ┌────┴────┐     ┌────┴────┐     ┌────┴────┐
      │DS18B20  │     │DS18B20  │     │DS18B20  │     │DS18B20  │     │DS18B20  │
      │Shock FL │     │Shock FR │     │Shock RL │     │Shock RR │     │Ambient  │
      └─────────┘     └─────────┘     └─────────┘     └─────────┘     └─────────┘


                    GPS MODULE (NEO-6M)
                    ═══════════════════

                         ┌───────────┐
          Raspberry Pi   │  NEO-6M   │
          GPIO 14 (TX) ──┤ RX        │
          GPIO 15 (RX) ──┤ TX        │
          3.3V ──────────┤ VCC       │
          GND ───────────┤ GND       │
                         └───────────┘


                    POWER DISTRIBUTION
                    ══════════════════

          Vehicle 12V ──────┬──────────────────┐
                            │                  │
                       ┌────┴────┐        ┌────┴────┐
                       │  Fuse   │        │  Fuse   │
                       │  5A     │        │  2A     │
                       └────┬────┘        └────┬────┘
                            │                  │
                       ┌────┴────┐             │
                       │  Buck   │             │
                       │Converter│             │
                       │12V → 5V │         ELM327
                       │   3A    │        (Powered
                       └────┬────┘         by OBD)
                            │
              ┌─────────────┼─────────────┐
              │             │             │
         Raspberry Pi   Sensors     GPS Module


                    POTENTIOMETER DETAIL
                    ════════════════════

                    Linear Potentiometer
                    (100mm stroke)
                         ┌─────────────────┐
          5V ────────────┤ VCC (Red)       │
                         │                 │
          ADS1115 A0 ────┤ Signal (White)  │
                         │                 │
          GND ───────────┤ GND (Black)     │
                         └─────────────────┘

                    Mounting:
                    ─────────
                    Bracket ───┬─── Shock Body (top)
                               │
                          Potentiometer
                               │
                    Rod End ───┴─── Lower A-Arm (bottom)
```

## Cable Specifications

| Cable | Type | Length | Notes |
|-------|------|--------|-------|
| I2C Bus | 4-conductor shielded | 2m max | Keep short for reliability |
| Potentiometer | 3-conductor shielded | 1.5m each | Shield to ground at one end |
| GPS | 4-conductor | 1m | Keep away from power cables |
| Temperature | 3-conductor | 2m each | Can daisy-chain sensors |
| Power | 16 AWG | As needed | Fused at source |

## Connector Recommendations

- **Weatherproof**: Deutsch DT series or equivalent
- **Indoor**: JST-XH or Molex for easy disconnection
- **Power**: Anderson Powerpole or XT60

## Grounding

Establish a single ground point (star ground):
1. Vehicle chassis ground at battery negative
2. All sensor grounds connect at Raspberry Pi GND
3. Shield drains connect at one end only (Pi end)
