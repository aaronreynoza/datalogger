# CAN Bus Wiring Guide for the ApexDirector Core Pro

**Audience:** Software developers with minimal electronics experience
**Hardware:** LilyGo T-Beam Supreme + SN65HVD230 CAN Transceiver Module
**Vehicle:** GWM Poer Luxury 2024 (or any vehicle with a standard OBD2 port)
**Last updated:** March 2026

---

## Table of Contents

1. [What is CAN Bus?](#1-what-is-can-bus)
2. [Understanding Each Component](#2-understanding-each-component)
3. [Breadboard Basics](#3-breadboard-basics)
4. [Step-by-Step Wiring Instructions](#4-step-by-step-wiring-instructions)
5. [OBD2 Connection](#5-obd2-connection)
6. [Power and Safety](#6-power-and-safety)
7. [Testing Checklist](#7-testing-checklist-before-starting-the-car)
8. [Troubleshooting](#8-troubleshooting)
9. [Complete Wiring Diagram](#9-complete-wiring-diagram)

---

## 1. What is CAN Bus?

Every modern car has dozens of tiny computers inside it. There is one controlling the
engine, another managing the transmission, one for the dashboard instruments, one for the
ABS brakes, and so on. These computers all need to talk to each other. For example, the
engine computer tells the dashboard computer the current RPM so the tachometer needle
knows where to point.

**CAN bus is the "party line telephone" that lets all these computers talk.** Imagine an
old-fashioned telephone line where everyone in the neighborhood shares one wire. Anyone
can pick up and listen, and anyone can talk -- but there are rules so people do not talk
over each other. That is exactly how CAN bus works. There is one shared pair of wires
running through the entire car, and every computer is connected to it. When the engine
computer wants to announce the current RPM, it puts a message on the wire. Every other
computer on the line hears it. The dashboard grabs it; the transmission grabs it; our
datalogger can grab it too.

Physically, CAN bus uses exactly **two wires** called **CAN High (CANH)** and
**CAN Low (CANL)**. The signal is the *voltage difference* between these two wires, which
makes it very resistant to electrical noise -- perfect for the harsh environment inside a
car. Our job is to connect our datalogger to these two wires so we can listen in on the
conversation.

---

## 2. Understanding Each Component

We need four things: the LilyGo T-Beam Supreme (our datalogger brain), the SN65HVD230
module (a translator between the datalogger and the car's CAN bus), a breadboard (a
plastic board for connecting wires without soldering), and an OBD2 extension cable (to
safely tap into the car's CAN wires). Let us go through each one.

---

### 2.1 LilyGo T-Beam Supreme (ESP32-S3)

This is the main board -- the "brain" of the datalogger. It has a GPS module, an IMU
(motion sensor), an SD card slot, and an ESP32-S3 microcontroller (a small computer on
a chip). For CAN bus wiring, we only care about four specific pins on this board.

**What the relevant pins do:**

| Pin Label | What It Does |
|-----------|-------------|
| **IO2 (GPIO 2)** | A general-purpose digital pin. We use it to **send** data to the CAN transceiver. |
| **IO3 (GPIO 3)** | A general-purpose digital pin. We use it to **receive** data from the CAN transceiver. |
| **DC1** | Outputs 3.3 volts of power (AXP2101 rail name). We use this to power the CAN transceiver module. |
| **GND** | Ground -- the common "zero point" for all electrical signals. Think of it as the return path for electricity. |

**Locating the pins on the board:**

The T-Beam Supreme has two long rows of pins (called "headers") running along both edges
of the board. The pin labels are printed in tiny white text on the board surface (called
the "silkscreen"). Hold the board with the USB-C port facing you, antenna pointing away.

```
                        ┌──── Antenna (GPS) ────┐
                        │                        │
    ╔═══════════════════╧════════════════════════╧═══════════════════╗
    ║                    LilyGo T-Beam Supreme                       ║
    ║                                                                ║
    ║   (left pin header)                    (right pin header)      ║
    ║                                                                ║
    ║    [GND] ●                                    ● [GND]          ║
    ║    [3V3] ●                                    ● [3V3]          ║
    ║          ●                                    ●                ║
    ║          ●                                    ●                ║
    ║   [IO 2] ●  <── GPIO 2 (CAN TX)              ●                ║
    ║   [IO 3] ●  <── GPIO 3 (CAN RX)              ●                ║
    ║          ●                                    ●                ║
    ║          ●                                    ●                ║
    ║          ●   ... more pins ...                ●                ║
    ║          ●                                    ●                ║
    ║                                                                ║
    ║          ┌────────────────────────────┐                        ║
    ║          │         ESP32-S3           │                        ║
    ║          │       (main chip)          │                        ║
    ║          └────────────────────────────┘                        ║
    ║                                                                ║
    ║                    ┌───────┐                                   ║
    ║                    │USB - C│                                   ║
    ╚════════════════════╧═══════╧═══════════════════════════════════╝
```

> **IMPORTANT:** Look for the silkscreen labels `IO2` and `IO3` on the expansion header.
> Do NOT use GPIO 4/5 — those are physically wired to the LoRa chip on the T-Beam Supreme.
> The power pin is labeled `DC1` (not "3V3") — it outputs 3.3V from the AXP2101 PMU.
> If you have trouble reading labels, take a close-up photo with your phone and zoom in.

> **TIP:** The board has GND and 3V3 pins in multiple locations. You can use any GND pin
> and any 3V3 pin. Choose whichever ones are most convenient for your wiring layout.

---

### 2.2 SN65HVD230 CAN Transceiver Module

This small breakout board (about the size of a postage stamp) is a **translator**. The
ESP32 speaks in 3.3V digital logic (just ones and zeros as voltage levels). The car's
CAN bus speaks in a special differential signal (the voltage *difference* between two
wires). The transceiver converts between these two languages.

Think of it like this: you speak English, the car speaks French, and the SN65HVD230 is
the interpreter sitting between you.

**The module typically looks like this:**

```
    ┌─────────────────────────────────┐
    │      SN65HVD230 CAN Module      │
    │                                  │
    │   ┌──────────────────────────┐   │
    │   │    SN65HVD230 chip       │   │
    │   │    (small black square)  │   │
    │   └──────────────────────────┘   │
    │                                  │
    │  [3V3] [GND] [CTX] [CRX] [CANH] [CANL]
    │    ●     ●     ●     ●     ●      ●
    └────┼─────┼─────┼─────┼─────┼──────┼──┘
         │     │     │     │     │      │
       Power Ground  │     │   To car  To car
       in    ref   TX in RX out (High) (Low)
       (from       (from  (to
       ESP32)     ESP32) ESP32)
```

> **NOTE:** Some modules have the pins in a different order, or may have an additional
> pin labeled **Rs**. The pin labels are printed on the board. Read them carefully.
> Different manufacturers lay out the pins differently.

**What each pin does:**

| Pin Label | Full Name | What It Does |
|-----------|-----------|-------------|
| **3V3** (or VCC) | Power Supply | Receives 3.3V power from the ESP32 to run the transceiver chip. |
| **GND** | Ground | Common ground reference. Must be shared with the ESP32. |
| **CTX** (or TX) | CAN Transmit Input | The ESP32 sends CAN data TO the transceiver through this pin. The transceiver then converts it to the differential CAN signal. |
| **CRX** (or RX) | CAN Receive Output | The transceiver receives CAN data FROM the car and sends a 3.3V version of it to the ESP32 through this pin. |
| **CANH** | CAN High | One of the two wires that connects to the car's CAN bus. Carries the "high" side of the differential signal. |
| **CANL** | CAN Low | The other wire that connects to the car's CAN bus. Carries the "low" side of the differential signal. |
| **Rs** (if present) | Slope Resistance | Controls the operating mode. Connecting it to GND enables "high-speed mode" which is what we want. |

> **NAMING CONFUSION:** The pin labeled "CTX" or "TX" on the module is an *input* from
> the ESP32's perspective. The ESP32 transmits data, and the transceiver receives it on
> CTX. Similarly, "CRX" or "RX" is an *output* -- the transceiver receives CAN data and
> outputs it to the ESP32. This is the opposite of what you might expect. Just remember:
> **ESP32 IO2 (TX) connects to the transceiver's driver input (CTX). ESP32 IO3
> (RX) connects to the transceiver's receiver output (CRX).**
>
> **If CAN shows `rx=0` and the loopback test (press `T` in serial monitor)
> fails, check for loose jumper wires first — this is the #1 wiring issue.**
> If the test still fails, try swapping IO2 and IO3 (some modules label pins
> from the bus perspective, reversing TX/RX).

---

### 2.3 OBD2 Connector

OBD2 (On-Board Diagnostics version 2) is a standardized diagnostic port found in every
car made after 1996. It is a trapezoid-shaped (wider on top, narrower on bottom) 16-pin
connector. The pins are arranged in two rows of 8.

**OBD2 connector pin numbering (looking at the MALE end, face-on):**

```
    Looking into the male connector end of the extension cable
    (the end with pins sticking out toward you)

              ╔═══════════════════════════════╗
             ╱  1   2   3   4   5   6   7   8  ╲
            ║   ●   ●   ●   ●   ●   ●   ●   ●  ║
            ║                                     ║
            ║   ●   ●   ●   ●   ●   ●   ●   ●  ║
             ╲  9  10  11  12  13  14  15  16   ╱
              ╚═══════════════════════════════╝

         Top row (wider):  Pins  1  through  8
         Bottom row:       Pins  9  through 16
```

**The pins we care about:**

| Pin | Name | What It Does | Do We Connect It? |
|-----|------|-------------|-------------------|
| **4** | Chassis Ground | The car's metal body ground reference. | YES -- connect to our common GND. |
| **5** | Signal Ground | Ground reference for the CAN signals. | YES -- connect to our common GND. |
| **6** | CAN High | The "high" wire of the CAN bus. | YES -- connect to transceiver CANH. |
| **14** | CAN Low | The "low" wire of the CAN bus. | YES -- connect to transceiver CANL. |
| **16** | Battery Positive | Direct 12V from the car battery. ALWAYS HOT (even with ignition off). | **NEVER CONNECT THIS.** |

> **WARNING: Pin 16 carries 12 volts directly from the car battery. It is always
> live, even when the car is off. NEVER connect pin 16 to anything in our circuit.
> 12 volts will instantly destroy the ESP32 and the CAN transceiver module, both of
> which run on 3.3 volts. If you are unsure which pin is which, triple-check with the
> diagram above before connecting anything.**

**How to identify pin numbers on the physical connector:**

1. Hold the male end of the OBD2 extension cable so you are looking directly at the pins.
2. The connector is a trapezoid -- the wider side is on top.
3. Pin 1 is at the top-left. Pin 8 is at the top-right.
4. Pin 9 is at the bottom-left. Pin 16 is at the bottom-right.
5. Some connectors have tiny numbers molded into the plastic. Use a flashlight to find
   them.
6. If there are no numbers, count from the left starting with pin 1 (top row) or pin 9
   (bottom row).

---

### 2.4 OBD2 Extension Cable

We use an extension cable (male-to-female) as a safe way to tap into the car's CAN bus
without cutting any factory wires.

```
    ┌──────────┐          Cable          ┌──────────┐
    │  Female  │ ======================== │   Male   │
    │   end    │    (passes all 16       │   end    │
    │          │     pins through)       │          │
    └──────────┘                         └──────────┘
         │                                     │
    Plugs INTO the                        We tap wires
    car's OBD2 port                       from HERE
    (under dashboard)
```

**How we use it:**

1. Plug the **female end** into the car's OBD2 port (under the dashboard).
2. The **male end** hangs free. This is where we attach our wires.
3. We connect jumper wires to pins 4, 5, 6, and 14 on the male end.
4. The car's normal diagnostics still work through the extension cable.

> **TIP:** To attach jumper wires to the male OBD2 pins, you can use female-to-male
> jumper wires. Push the female end of a jumper wire onto the exposed male pin of the
> OBD2 connector. It should grip snugly. If it feels loose, wrap a tiny piece of tape
> around the connection to keep it secure. For a more permanent setup, you can later
> solder wires to an OBD2 breakout board, but jumper wires are fine for prototyping.

---

### 2.5 Breadboard

A breadboard is a plastic board with hundreds of small holes arranged in a grid. You push
wires and component pins into the holes to make electrical connections. No soldering
needed. We will use it as the central "meeting point" where all our wires come together.

Breadboards are explained in full detail in the next section.

---

### 2.6 Jumper Wires

Jumper wires are short, flexible wires with connectors on each end. They come in three
types:

| Type | Ends | When to Use |
|------|------|-------------|
| **Male-to-male** | Pin on both ends | Connecting two breadboard holes, or two pin headers. |
| **Male-to-female** | Pin on one end, socket on the other | Connecting a board's pin header to a breadboard. |
| **Female-to-female** | Socket on both ends | Connecting two board pin headers directly. |

For this project, you will primarily need **male-to-female** wires (to connect the
T-Beam's pin headers to the breadboard) and **male-to-male** wires (for connections
within the breadboard).

---

## 3. Breadboard Basics

If you have never used a breadboard before, this section explains how they work. If you
already know, skip to Section 4.

### 3.1 How a Breadboard Works

A breadboard has a grid of holes. Underneath the surface, certain holes are connected
together by metal strips. The key rule is:

- **Holes in the same row (in the main area) are connected together.**
- **Holes in different rows are NOT connected.**
- **The power rails (along the edges) run the entire length of the board.**

### 3.2 Breadboard Layout Diagram

```
    Power Rails                Main Area                Power Rails
    (vertical)             (horizontal rows)            (vertical)

    +    -          a  b  c  d  e     f  g  h  i  j          +    -
    |    |          ─────────────     ─────────────           |    |
    ●    ●     1    ●  ●  ●  ●  ●     ●  ●  ●  ●  ●    1    ●    ●
    |    |          ─────────────     ─────────────           |    |
    ●    ●     2    ●  ●  ●  ●  ●     ●  ●  ●  ●  ●    2    ●    ●
    |    |          ─────────────     ─────────────           |    |
    ●    ●     3    ●  ●  ●  ●  ●     ●  ●  ●  ●  ●    3    ●    ●
    |    |          ─────────────     ─────────────           |    |
    ●    ●     4    ●  ●  ●  ●  ●     ●  ●  ●  ●  ●    4    ●    ●
    |    |          ─────────────     ─────────────           |    |
    ●    ●     5    ●  ●  ●  ●  ●     ●  ●  ●  ●  ●    5    ●    ●
    |    |                                                    |    |
    ●    ●    ...   (more rows)                 (more rows)   ●    ●
    |    |                                                    |    |
    ●    ●    30    ●  ●  ●  ●  ●     ●  ●  ●  ●  ●   30    ●    ●
    |    |          ─────────────     ─────────────           |    |
```

**Key points:**

1. **Main area rows:** In row 1, holes a1-b1-c1-d1-e1 are all connected together
   (one metal strip underneath). Holes f1-g1-h1-i1-j1 are also connected together
   (a separate strip). But a1-e1 are NOT connected to f1-j1 -- the center gap
   (called the "ravine") separates them.

2. **Power rails:** The columns marked `+` and `-` on each side run vertically.
   All holes in the `+` column are connected. All holes in the `-` column are
   connected. These are for distributing power (3.3V on `+`) and ground (on `-`).

3. **The center gap:** The gap between columns e and f exists so that you can plug
   in DIP chips (integrated circuits in wide rectangular packages) that straddle the
   gap. We are not using DIP chips, but the gap is still there.

### 3.3 Practical Example

If you push a wire into hole `a3` and another wire into hole `d3`, those two wires are
electrically connected (because they are in the same row on the same side of the gap).

If you push a wire into hole `a3` and another into hole `a5`, they are NOT connected
(different rows).

```
    Connected:       a3 ●──●──●──●──● e3     (same row, same side)
                       a  b  c  d  e

    NOT connected:   a3 ●                     (different rows)
                     a5 ●

    NOT connected:   e3 ●     ● f3            (gap separates them)
```

### 3.4 Setting Up Power Rails

Before wiring components, we will connect the T-Beam's 3V3 and GND pins to the
breadboard's power rails. This gives us a convenient "bus" to distribute power to
multiple components.

```
    + rail  ────────  3.3V from ESP32 (red wire by convention)
    - rail  ────────  GND from ESP32 (black wire by convention)
```

Once connected, any component that needs power can tap into the `+` rail for 3.3V
and the `-` rail for ground, without running wires all the way back to the T-Beam.

---

## 4. Step-by-Step Wiring Instructions

We will make **8 wire connections** in total. Go slowly. Check each wire before moving
to the next one. Use the color suggestions to keep things organized (but any color will
work electrically -- colors are just for your sanity).

### Overview of All Connections

| Wire # | From | To | Color | Purpose |
|--------|------|----|-------|---------|
| 1 | T-Beam **DC1** (3.3V) | Breadboard **+ rail** | Red | Power bus |
| 2 | T-Beam **GND** | Breadboard **- rail** | Black | Ground bus |
| 3 | Breadboard **+ rail** | SN65HVD230 **3V3** | Red | Power to transceiver |
| 4 | Breadboard **- rail** | SN65HVD230 **GND** | Black | Ground to transceiver |
| 5 | T-Beam **IO2** | SN65HVD230 **CTX** (TX) | Yellow | CAN transmit signal |
| 6 | T-Beam **IO3** | SN65HVD230 **CRX** (RX) | Green | CAN receive signal |
| 7 | SN65HVD230 **Rs** | Breadboard **- rail** (GND) | Black | Enable high-speed mode |
| 8a | SN65HVD230 **CANH** | OBD2 **Pin 6** | White | CAN High to car |
| 8b | SN65HVD230 **CANL** | OBD2 **Pin 14** | Blue | CAN Low to car |
| 8c | OBD2 **Pin 4** | Breadboard **- rail** (GND) | Black | Chassis ground |
| 8d | OBD2 **Pin 5** | Breadboard **- rail** (GND) | Black | Signal ground |

---

### Preparation

1. **Clear your workspace.** Use a flat, well-lit surface. A desk lamp helps.
2. **Place the breadboard** in front of you with row numbers visible.
3. **Place the SN65HVD230 module** into the breadboard. Push its pins into one row
   of holes. Each pin should go into a different row so they are not shorted together.
   For example, if the module has 6 pins in a line, place them in rows 10 through 15.
4. **Place the T-Beam Supreme** next to the breadboard. It does NOT go into the
   breadboard (it is too big). It sits beside it, and we run wires between them.
5. **Have your jumper wires ready.** Sort them by color if possible.

```
    ┌──────────────────────────────────────────────────────────────────┐
    │                         YOUR DESK                                │
    │                                                                  │
    │   ┌──────────────────┐        ┌────────────────────────────┐    │
    │   │   T-Beam Supreme │        │        Breadboard          │    │
    │   │                  │        │                            │    │
    │   │  (USB-C facing   │  wires │  SN65HVD230 plugged in    │    │
    │   │   toward you)    │ =====> │  around rows 10-15        │    │
    │   │                  │        │                            │    │
    │   └──────────────────┘        └────────────────────────────┘    │
    │                                                                  │
    │   USB cable to laptop                  Wires to OBD2 cable      │
    │   (power + serial)                     (goes to the car)        │
    └──────────────────────────────────────────────────────────────────┘
```

---

### Wire 1: Power Bus (3V3)

**Purpose:** Bring 3.3V power from the T-Beam to the breadboard's power rail.

1. Take a **red** male-to-female jumper wire.
2. Push the **female end** onto the T-Beam's pin labeled **3V3** (3.3 volt output).
3. Push the **male end** into any hole on the breadboard's **+ (positive) power rail**
   (the column marked with a red line or `+` symbol, running along the edge).

```
    T-Beam [3V3] ───── red wire ────── Breadboard [+ rail]
```

---

### Wire 2: Ground Bus (GND)

**Purpose:** Connect the T-Beam's ground to the breadboard's ground rail. This creates
a common ground reference shared by all components.

1. Take a **black** male-to-female jumper wire.
2. Push the **female end** onto the T-Beam's pin labeled **GND**.
3. Push the **male end** into any hole on the breadboard's **- (negative) power rail**
   (the column marked with a blue or black line or `-` symbol).

```
    T-Beam [GND] ───── black wire ───── Breadboard [- rail]
```

> **WHY GROUND MATTERS:** Every electronic circuit needs a common reference point for
> "zero volts." If the ESP32 and the CAN transceiver do not share a common ground,
> they cannot understand each other's signals. Think of it like two people trying to
> measure height -- they need to agree on where "sea level" is. Ground is electrical
> sea level.

---

### Wire 3: Power to CAN Transceiver (3V3)

**Purpose:** Supply 3.3V power to the SN65HVD230 module from the breadboard's power rail.

1. Take a **red** male-to-male jumper wire.
2. Push one end into the breadboard's **+ power rail**.
3. Push the other end into the breadboard **row where the SN65HVD230's 3V3 (or VCC)
   pin is inserted**.

   (For example, if you plugged the module's 3V3 pin into row 10, column `a`, then
   push this wire into row 10, column `b` or `c` or `d` or `e` -- any hole in the
   same row on the same side of the center gap.)

```
    Breadboard [+ rail] ───── red wire ───── Breadboard [row of module 3V3 pin]
                                                    │
                                              (connected to SN65HVD230 3V3 pin
                                               because they share the same row)
```

---

### Wire 4: Ground to CAN Transceiver (GND)

**Purpose:** Connect the SN65HVD230 module's ground to the common ground bus.

1. Take a **black** male-to-male jumper wire.
2. Push one end into the breadboard's **- power rail**.
3. Push the other end into the breadboard **row where the SN65HVD230's GND pin is
   inserted**.

```
    Breadboard [- rail] ───── black wire ───── Breadboard [row of module GND pin]
                                                    │
                                              (connected to SN65HVD230 GND pin)
```

---

### Wire 5: CAN Transmit (IO2 to CTX)

**Purpose:** This wire carries the CAN transmit signal from the ESP32 to the
transceiver. When the ESP32 wants to send a CAN message, the signal travels through
this wire.

1. Take a **yellow** male-to-female jumper wire.
2. Push the **female end** onto the T-Beam's pin labeled **IO2** (GPIO 2).
3. Push the **male end** into the breadboard **row where the SN65HVD230's CTX
   (or TX) pin is inserted**.

```
    T-Beam [IO2] ───── yellow wire ───── Breadboard [row of module CTX pin]
                                                    │
                                              (connected to SN65HVD230 CTX)
```

> **DOUBLE-CHECK:** IO**2** goes to **CTX** (transmit). Not IO3. Not CRX.
> Getting these swapped is the most common wiring mistake and will result in no
> communication. Do NOT use GPIO 4/5 — those are wired to the LoRa chip.

---

### Wire 6: CAN Receive (IO3 to CRX)

**Purpose:** This wire carries the CAN receive signal from the transceiver to the
ESP32. When a car computer sends a CAN message, the transceiver converts it and sends
it to the ESP32 through this wire.

1. Take a **green** male-to-female jumper wire.
2. Push the **female end** onto the T-Beam's pin labeled **IO3** (GPIO 3).
3. Push the **male end** into the breadboard **row where the SN65HVD230's CRX
   (or RX) pin is inserted**.

```
    T-Beam [IO3] ───── green wire ───── Breadboard [row of module CRX pin]
                                                    │
                                              (connected to SN65HVD230 CRX)
```

---

### Wire 7: Rs Pin to Ground (High-Speed Mode)

**Purpose:** If your SN65HVD230 module has a pin labeled **Rs**, connecting it to
ground enables high-speed mode (up to 1 Mbps). This is the normal operating mode for
automotive CAN bus.

1. **First, check if your module has an Rs pin.** Look at the pin labels. If there is
   no Rs pin, **skip this wire entirely** -- your module handles this internally.
2. If Rs exists: take a short **black** male-to-male jumper wire.
3. Push one end into the breadboard **row where the SN65HVD230's Rs pin is inserted**.
4. Push the other end into the breadboard's **- (ground) rail**.

```
    Breadboard [row of module Rs pin] ───── black wire ───── Breadboard [- rail]
```

> **NOTE:** If the Rs pin is left unconnected ("floating"), the transceiver may enter
> a low-power or slope-controlled mode, which can cause communication failures. When
> in doubt, connect it to ground.

---

### Pause and Verify (Before Car Connection)

At this point, you have wired everything between the T-Beam and the SN65HVD230 module.
Before connecting to the car, let us verify:

**Verification Checklist -- Board-Side Wiring:**

- [ ] Red wire from T-Beam DC1 (3.3V) to breadboard + rail
- [ ] Black wire from T-Beam GND to breadboard - rail
- [ ] Red wire from breadboard + rail to the row of the module's 3V3/VCC pin
- [ ] Black wire from breadboard - rail to the row of the module's GND pin
- [ ] Yellow wire from T-Beam IO2 to the row of the module's CTX/TX pin
- [ ] Green wire from T-Beam IO3 to the row of the module's CRX/RX pin
- [ ] (If Rs pin exists) Black wire from the row of the module's Rs pin to - rail
- [ ] No wires are touching each other where they should not be
- [ ] The SN65HVD230 module is firmly seated in the breadboard (pins fully pushed in)
- [ ] Each module pin is in a **different** row (no two pins share a row)

> **CAUTION:** If two module pins are accidentally in the same breadboard row, they
> are electrically shorted (connected) together. This could damage components. Check
> that every pin of the module goes into its own unique row.

---

### Wire 8: Car Connection (OBD2)

This step connects the SN65HVD230 transceiver to the car's CAN bus through the OBD2
extension cable. **Do these connections with the car OFF and the extension cable NOT
plugged into the car.**

#### Wire 8a: CAN High

1. Take a **white** jumper wire (male-to-male or male-to-female as needed).
2. Connect one end to the breadboard **row where the SN65HVD230's CANH pin is
   inserted**.
3. Connect the other end to **Pin 6** on the **male end** of the OBD2 extension cable.

```
    Breadboard [row of CANH pin] ───── white wire ───── OBD2 male Pin 6
```

**Finding Pin 6:** Look at the male end of the OBD2 cable face-on. Pin 6 is in the
top row, 6th from the left.

```
    Top row:    1   2   3   4   5  [6]  7   8
                                    ▲
                                    │
                              CAN High (Pin 6)
```

#### Wire 8b: CAN Low

1. Take a **blue** jumper wire.
2. Connect one end to the breadboard **row where the SN65HVD230's CANL pin is
   inserted**.
3. Connect the other end to **Pin 14** on the **male end** of the OBD2 extension cable.

```
    Breadboard [row of CANL pin] ───── blue wire ───── OBD2 male Pin 14
```

**Finding Pin 14:** Pin 14 is in the bottom row, 6th from the left.

```
    Bottom row:  9  10  11  12  13 [14] 15  16
                                    ▲
                                    │
                              CAN Low (Pin 14)
```

> **MEMORY AID:** Pin 6 and Pin 14 are directly above/below each other (both 6th from
> the left). CAN High is on top (pin 6), CAN Low is on the bottom (pin 14). "High is
> on top."

#### Wire 8c: Chassis Ground

1. Take a **black** jumper wire.
2. Connect one end to the breadboard's **- (ground) rail**.
3. Connect the other end to **Pin 4** on the male end of the OBD2 extension cable.

```
    Breadboard [- rail] ───── black wire ───── OBD2 male Pin 4
```

**Finding Pin 4:** Top row, 4th from the left.

#### Wire 8d: Signal Ground

1. Take a **black** jumper wire.
2. Connect one end to the breadboard's **- (ground) rail**.
3. Connect the other end to **Pin 5** on the male end of the OBD2 extension cable.

```
    Breadboard [- rail] ───── black wire ───── OBD2 male Pin 5
```

**Finding Pin 5:** Top row, 5th from the left.

> **WHY TWO GROUND PINS?** Pin 4 is "chassis ground" (connected to the car's metal
> body) and Pin 5 is "signal ground" (the reference for CAN signals). Connecting both
> ensures a solid ground reference. Some cars only use one or the other; connecting
> both covers all cases.

---

### Final Verification Checklist -- Complete Wiring

Go through every single connection one more time:

- [ ] **Wire 1:** T-Beam DC1 (3.3V) --> breadboard + rail (red)
- [ ] **Wire 2:** T-Beam GND --> breadboard - rail (black)
- [ ] **Wire 3:** Breadboard + rail --> SN65HVD230 3V3/VCC row (red)
- [ ] **Wire 4:** Breadboard - rail --> SN65HVD230 GND row (black)
- [ ] **Wire 5:** T-Beam IO2 --> SN65HVD230 CTX/TX row (yellow)
- [ ] **Wire 6:** T-Beam IO3 --> SN65HVD230 CRX/RX row (green)
- [ ] **Wire 7:** SN65HVD230 Rs row --> breadboard - rail (black) -- if Rs pin exists
- [ ] **Wire 8a:** SN65HVD230 CANH row --> OBD2 Pin 6 (white)
- [ ] **Wire 8b:** SN65HVD230 CANL row --> OBD2 Pin 14 (blue)
- [ ] **Wire 8c:** OBD2 Pin 4 --> breadboard - rail (black)
- [ ] **Wire 8d:** OBD2 Pin 5 --> breadboard - rail (black)
- [ ] **NO wire connected to OBD2 Pin 16** (12V battery -- do NOT touch)
- [ ] All wires are firmly seated (give each a gentle tug)
- [ ] No bare wire ends are touching anything they should not

---

## 5. OBD2 Connection

### 5.1 Locating the OBD2 Port in the GWM Poer Luxury 2024

The OBD2 port in the GWM Poer is located **under the dashboard on the driver's side**.

1. Sit in the driver's seat.
2. Look underneath the dashboard, below and to the left of the steering column.
3. The port is a trapezoid-shaped 16-pin connector, usually black plastic.
4. It may be behind a small plastic cover that snaps off.
5. It faces downward or toward the driver's knees.

```
    ┌─────────────────────────────────────────────┐
    │                  Dashboard                   │
    │                                              │
    │    Steering                                  │
    │    Column                                    │
    │      │                                       │
    │      │    ┌───────┐                          │
    │      │    │ OBD2  │  <── Look here           │
    │      ▼    │ port  │      (under dash,        │
    │           └───────┘       driver side)        │
    │                                              │
    │           ══════════════                     │
    │           (driver's knees)                   │
    └─────────────────────────────────────────────┘
```

> **TIP:** Use your phone's flashlight. The port can be hard to see in the dark space
> under the dashboard.

### 5.2 Using the Extension Cable

1. **With the car OFF**, plug the **female end** of the OBD2 extension cable into the
   car's OBD2 port. The trapezoid shape means it can only go in one way. Push firmly
   until it clicks or seats fully.

2. The **male end** of the extension cable should now hang down accessible. This is
   where our wires (8a, 8b, 8c, 8d) connect.

3. Route the cable so it does not interfere with the pedals or steering. Tuck it along
   the side of the center console.

```
    Car's OBD2 port
         │
         ▼
    ┌──────────┐
    │  Female  │ <── plugged into car
    │   end    │
    └─────┬────┘
          │
          │  extension cable
          │
    ┌─────┴────┐
    │   Male   │ <── our wires connect here (Pins 4, 5, 6, 14)
    │   end    │
    └──────────┘
          │
          │  jumper wires to breadboard
          │
    ┌─────┴──────────────────┐
    │      Breadboard         │
    │   (with SN65HVD230)    │
    └────────────────────────┘
```

### 5.3 Pin Identification Tips for the OBD2 Male End

The male end of the extension cable has 16 pins protruding outward. To identify which
pin is which:

```
    Step 1: Hold the male connector so you're looking at the pins.
    Step 2: Orient it so the wider edge is on TOP.
    Step 3: The pins are numbered:

              ╔══════════════════════════════╗
             ╱  1   2   3  [4] [5] [6]  7   8  ╲       <- Top row (wider)
            ║   ●   ●   ●   ●   ●   ●   ●   ●  ║
            ║                                     ║
            ║   ●   ●   ●   ●   ●   ●   ●   ●  ║
             ╲  9  10  11  12  13 [14] 15  16   ╱       <- Bottom row
              ╚══════════════════════════════╝

            [4]  = Chassis Ground     ─── connect to GND rail
            [5]  = Signal Ground      ─── connect to GND rail
            [6]  = CAN High           ─── connect to CANH
            [14] = CAN Low            ─── connect to CANL
```

> **WARNING: Pin 16 (bottom row, far right) is 12V battery power. It is ALWAYS
> energized, even with the car off and keys removed. Do not touch it with any wire.
> Do not let any loose jumper wire ends dangle near it.**

---

## 6. Power and Safety

### 6.1 How Everything Gets Power

There are **two completely separate power systems** in this setup:

```
    POWER SYSTEM 1: Our electronics (3.3V)
    ┌──────────────────────────────────────────────────────┐
    │                                                      │
    │   Laptop/Power Bank ──USB-C──> T-Beam Supreme        │
    │                                    │                 │
    │                                    │ 3.3V pin        │
    │                                    ▼                 │
    │                               SN65HVD230 module      │
    │                                                      │
    │   Everything runs on 3.3V from the ESP32's regulator │
    └──────────────────────────────────────────────────────┘

    POWER SYSTEM 2: The car (12V)
    ┌──────────────────────────────────────────────────────┐
    │                                                      │
    │   Car Battery (12V) ──> Car's CAN bus network        │
    │                         (engine ECU, ABS, dash, etc) │
    │                                                      │
    │   We only connect to the CAN signal wires            │
    │   (CANH + CANL) and ground (Pin 4 + Pin 5).         │
    │   We do NOT take power from the car.                 │
    └──────────────────────────────────────────────────────┘
```

**Key points:**

- The ESP32 T-Beam gets its power from **USB** (your laptop or a USB power bank).
  **It does NOT get power from the car.**
- The SN65HVD230 module gets 3.3V from the **ESP32's 3V3 pin**.
  **It does NOT get power from the car.**
- We connect to the car ONLY for the CAN signal wires and ground. No power flows
  from the car to our electronics.

### 6.2 Safety Rules

> **RULE 1: NEVER connect OBD2 Pin 16 to anything.**
> Pin 16 is direct 12V from the car battery. It will destroy the ESP32 (which runs
> on 3.3V) and the CAN transceiver instantly. There is no protection circuit.

> **RULE 2: The CAN bus is electrically isolated from high-power circuits.**
> The CAN bus wires (CANH, CANL) carry only small signals (typically 0V to 3.5V).
> You cannot damage the car's engine, transmission, or any critical system by
> reading the CAN bus. The transceiver only *listens* -- it does not send commands
> that could affect car operation (we configure it in listen-only mode).

> **RULE 3: Always connect the ESP32 via USB BEFORE plugging the OBD2 cable into
> the car.** This ensures the transceiver is powered and its input pins are at known
> voltage levels. Connecting unpowered electronics to a live bus can sometimes cause
> brief voltage spikes on pins that are not yet driven.

> **RULE 4: When in doubt, disconnect and re-check.**
> If anything looks wrong, smells wrong, or feels hot -- unplug the USB cable and
> remove the OBD2 extension cable immediately. Then re-check all wiring.

### 6.3 What Could Go Wrong (and Why It Probably Will Not)

| Scenario | Risk Level | What Happens |
|----------|-----------|-------------|
| CANH and CANL swapped | **Low** | No communication. No damage. Just swap the wires. |
| IO2 and IO3 swapped | **Low** | No communication. No damage. Just swap the wires. |
| 3V3 and GND swapped on module | **Medium** | Could damage the SN65HVD230 module. Double-check before powering on. |
| Pin 16 (12V) connected to ESP32 | **HIGH** | ESP32 destroyed. SN65HVD230 destroyed. Avoid at all costs. |
| Loose wire falls on Pin 16 | **HIGH** | Same as above. Keep loose wires tucked away. |
| Module not in breadboard firmly | **None** | Just no communication. Re-seat the module. |

---

## 7. Testing Checklist (Before Starting the Car)

### 7.1 Visual Inspection (No Power)

Do this inspection BEFORE plugging in the USB cable or the OBD2 cable.

- [ ] All wires match the connection table in Section 4
- [ ] No bare wire ends dangling loose
- [ ] No two adjacent OBD2 pins accidentally bridged by a single wire
- [ ] The SN65HVD230 module is firmly seated in the breadboard
- [ ] Each module pin is in a DIFFERENT breadboard row
- [ ] No wire connects to OBD2 Pin 16

### 7.2 Multimeter Checks (If You Have One)

If you own a multimeter (a small tool that measures voltage and checks wire
connections), these tests will catch wiring errors before they cause problems.
If you do not have a multimeter, skip to 7.3 -- the visual inspection is usually
sufficient.

**Set your multimeter to "Continuity" mode** (usually marked with a speaker/buzzer
symbol). In this mode, the multimeter beeps when the two probes are touching things
that are electrically connected.

**Test 1: Verify ground connections**
- Touch one probe to the T-Beam's GND pin.
- Touch the other probe to the SN65HVD230 module's GND pin.
- **Expected:** BEEP (continuity). They share the same ground through the breadboard.
- If no beep: check Wires 2 and 4.

**Test 2: Verify power connections**
- Touch one probe to the T-Beam's 3V3 pin.
- Touch the other probe to the SN65HVD230 module's 3V3/VCC pin.
- **Expected:** BEEP.
- If no beep: check Wires 1 and 3.

**Test 3: Verify no short between 3V3 and GND**
- Touch one probe to the breadboard + rail.
- Touch the other probe to the breadboard - rail.
- **Expected:** NO beep (no continuity). If it beeps, there is a short circuit.
  Find and fix it before powering anything on.

**Test 4: Verify OBD2 ground connections**
- Touch one probe to the breadboard - rail.
- Touch the other probe to OBD2 Pin 4.
- **Expected:** BEEP.
- Repeat with OBD2 Pin 5. Also expected: BEEP.

**Test 5: Verify no connection to Pin 16**
- Touch one probe to the breadboard + rail.
- Touch the other probe to OBD2 Pin 16.
- **Expected:** NO beep. If it beeps, a wire is in the wrong place. STOP and fix it.
- Repeat with the probe on the breadboard - rail. Also expected: NO beep.

### 7.3 Power-On Test (USB Only, No Car)

1. Plug the USB-C cable into the T-Beam Supreme and into your laptop.
2. The T-Beam should power on (you may see a small LED light up).
3. **Do NOT plug the OBD2 cable into the car yet.**
4. Open a serial monitor:
   ```
   pio device monitor -b 115200
   ```
5. Look for the CAN bus initialization message in the serial output. Once the TWAI
   (Two-Wire Automotive Interface -- Espressif's name for their CAN controller) driver
   is implemented, you should see:
   ```
   CAN bus: TWAI started: LISTEN mode, 500 kbps
   ```
   If you see this, the ESP32 successfully initialized the CAN controller and
   transceiver.

6. **If you see `CAN bus: not connected (stub)`**: The firmware still has the stub
   CAN implementation. This means the wiring is not being tested yet by firmware.
   The firmware needs to be updated to use the ESP32's TWAI driver. The wiring is
   still correct -- the firmware just needs the CAN code written.

7. Check that nothing on the breadboard is getting hot. Touch the SN65HVD230 module
   gently. It should be at room temperature or barely warm. If anything is hot,
   **immediately unplug USB** and re-check wiring (likely 3V3 and GND swapped).

### 7.4 Full Test (With Car)

1. Ensure the T-Beam is already powered via USB (from Section 7.3).
2. Plug the **female end** of the OBD2 extension cable into the car's OBD2 port.
3. **Turn the car's ignition to ON (accessory mode or engine on).** CAN bus messages
   are usually only sent when the ignition is on.
4. Watch the serial monitor.

**Expected behavior (once firmware CAN code is active):**

```
CAN bus: TWAI started: LISTEN mode, 500 kbps
CAN: frame ID=0x123 len=8 data=[01 02 03 04 05 06 07 08]
CAN: frame ID=0x456 len=8 data=[AA BB CC DD 00 00 00 00]
CAN: frame ID=0x123 len=8 data=[01 02 04 04 05 06 07 08]
...
```

You should see a stream of CAN frames with various IDs. A typical car sends hundreds
of messages per second. If messages appear, everything is wired correctly.

**If no messages appear after 10 seconds:**
- See the Troubleshooting section below.

---

## 8. Troubleshooting

### 8.1 No CAN Frames Received

**Symptom:** Serial monitor shows the TWAI driver started, but no frames appear.

**Check in this order:**

1. **Is the car's ignition ON?** CAN messages are typically not sent when the car is
   fully off. Turn the key to the ON position (or press the start button without
   pressing the brake, for push-button start cars).

2. **Is the baud rate correct?** Most modern cars use 500 kbps (kilobits per second)
   for the main CAN bus. The GWM Poer should use 500 kbps. If the firmware is
   configured for a different speed, no frames will be received. Check the firmware
   configuration.

3. **Are CANH and CANL wires correct?** Try swapping the white (CANH) and blue (CANL)
   wires. Some extension cables have non-standard internal wiring. Swapping CAN High
   and CAN Low will not damage anything.

4. **Are IO2 and IO3 correct?** Verify that IO2 goes to CTX (not CRX) and
   IO3 goes to CRX (not CTX). Swapping these will not cause damage but will
   prevent communication. Do NOT use GPIO 4/5 — those are wired to the LoRa chip.

5. **Is the SN65HVD230 module getting power?** Check that its 3V3 pin has 3.3V
   (measure with a multimeter between the module's 3V3 and GND pins). If 0V, check
   Wires 1 and 3.

6. **Is the Rs pin connected to GND?** If it is floating, the transceiver may not be
   in the correct mode.

7. **Is the car's gateway blocking OBD2 CAN access?** Some modern vehicles have a
   "gateway" module that filters CAN messages and prevents them from reaching the
   OBD2 port. This is a security feature. If the GWM Poer has a gateway, you may
   need to tap into the CAN bus at a different point (directly at the engine ECU or
   at the CAN bus backbone behind the dashboard). This is a more advanced procedure.

8. **Try a different OBD2 extension cable.** Some cheap cables have poor internal
   connections.

### 8.2 Bus Errors on Serial Monitor

**Symptom:** Serial monitor shows error messages like "BUS_ERROR" or "ERR_PASS" or
error counters incrementing.

**Possible causes:**

1. **Wiring issue:** A loose connection on CANH or CANL can cause intermittent errors.
   Check that all wires are firmly seated.

2. **Missing ground:** If Wires 8c and 8d (OBD2 Pin 4 and Pin 5 to GND rail) are not
   connected, the CAN signal has no ground reference and will produce errors.

3. **Wrong baud rate:** If the firmware CAN speed does not match the car's CAN speed,
   the controller will see the data as corrupted and report errors.

4. **No termination needed:** A proper CAN bus has 120-ohm termination resistors at
   each end of the bus. The car already has these built in. **You do NOT need to add
   a termination resistor.** Some SN65HVD230 modules come with a 120-ohm resistor
   built in (sometimes as a solder jumper). If your module has this and the car
   already has termination, the extra resistance might cause minor signal issues. If
   you see persistent errors, check if your module has a termination resistor and
   remove it (cut the solder jumper or desolder the resistor).

### 8.3 ESP32 Keeps Rebooting

**Symptom:** The serial monitor shows the ESP32 boot message repeating every few
seconds, possibly with a crash backtrace.

**Possible causes:**

1. **GND not connected:** Without a shared ground, the CAN transceiver's output pin
   (CRX) can float to random voltages, potentially causing the ESP32's TWAI
   controller to malfunction and trigger the watchdog timer.

2. **Short circuit:** Two wires or pins are touching that should not be. Check that
   no adjacent breadboard rows are accidentally bridged.

3. **Firmware bug:** If the reboot happens even without the OBD2 cable plugged into
   the car, the issue is likely in firmware, not wiring. Check the serial output for
   a backtrace and decode it with `addr2line` to find the exact function causing the
   crash.

4. **Power issue:** If the T-Beam is running on a weak USB power source (some laptop
   USB ports only provide 500mA), the added current draw of the CAN transceiver
   (small, but nonzero) might cause a brownout. Try using a powered USB hub or a
   USB wall adapter rated for at least 1A.

### 8.4 Serial Monitor Shows "CAN bus: not connected (stub)"

This is not a wiring problem. The firmware currently has a placeholder ("stub")
implementation for CAN bus. The wiring is correct, but the firmware code in
`can_bus.cpp` needs to be updated to use the ESP32's TWAI driver. The current stub
always returns "not connected" regardless of wiring.

### 8.5 Module Gets Hot

If the SN65HVD230 module feels hot to the touch:

1. **Immediately unplug the USB cable** from the T-Beam.
2. **Remove the OBD2 cable** from the car.
3. Check if 3V3 and GND are swapped on the module (Wires 3 and 4). Reversing power
   and ground will damage the chip and cause it to heat up.
4. Check if 12V from OBD2 Pin 16 accidentally reached the module.
5. If 3V3/GND were swapped, the module may be damaged. Replace it (they cost about
   $2-3 each).

---

## 9. Complete Wiring Diagram

This diagram shows every component and every wire. Wire numbers match Section 4.

```
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                           COMPLETE WIRING DIAGRAM                          │
    │                                                                             │
    │                                                                             │
    │   T-Beam Supreme                 Breadboard                                │
    │   ┌──────────────┐              ┌──────────────────────────────────┐        │
    │   │              │              │  + rail (3.3V)   - rail (GND)   │        │
    │   │         3V3 ●┤── Wire 1 ──>├─●+ + + + + + + + + + + + + +   │        │
    │   │              │  (red)       │                                  │        │
    │   │         GND ●┤── Wire 2 ──>├─               ●- - - - - - - - │        │
    │   │              │  (black)     │                │                 │        │
    │   │              │              │                │                 │        │
    │   │         IO2 ●┤── Wire 5 ──>├──────┐         │                 │        │
    │   │              │  (yellow)    │      │         │                 │        │
    │   │         IO3 ●┤── Wire 6 ──>├────┐ │         │                 │        │
    │   │              │  (green)     │    │ │         │                 │        │
    │   │              │              │    │ │         │                 │        │
    │   │   [USB-C]    │              │    │ │         │                 │        │
    │   └──────┬───────┘              │    │ │         │                 │        │
    │          │                      │    │ │         │                 │        │
    │          │ USB to laptop        │  SN65HVD230 Module              │        │
    │          │ (power + serial)     │  (plugged into breadboard)      │        │
    │          ▼                      │    │ │         │                 │        │
    │       Laptop                    │  Row  10: [3V3]●──Wire 3──●+ rail       │
    │                                 │            │   (red)        │   │        │
    │                                 │  Row  11: [GND]●──Wire 4──●- rail       │
    │                                 │            │   (black)     │   │        │
    │                                 │  Row  12: [CTX]●───────────┘   │        │
    │                                 │            │  (Wire 5, yellow)  │        │
    │                                 │  Row  13: [CRX]●───────────┘   │        │
    │                                 │            │  (Wire 6, green)   │        │
    │                                 │  Row  14: [CANH]●──Wire 8a ──>│── to OBD2 Pin 6  │
    │                                 │            │   (white)         │        │
    │                                 │  Row  15: [CANL]●──Wire 8b ──>│── to OBD2 Pin 14 │
    │                                 │            │   (blue)          │        │
    │                                 │  Row  16:  [Rs]●──Wire 7──●- rail      │
    │                                 │                  (black)       │        │
    │                                 │                                │        │
    │                                 │  - rail ●────Wire 8c ────────>│── to OBD2 Pin 4  │
    │                                 │         │    (black)           │        │
    │                                 │  - rail ●────Wire 8d ────────>│── to OBD2 Pin 5  │
    │                                 │              (black)           │        │
    │                                 └────────────────────────────────┘        │
    │                                                                           │
    │                                                                           │
    │                           OBD2 Extension Cable                            │
    │                                                                           │
    │    Male End (wires connect here)              Female End                  │
    │    ┌─────────────────────────┐                ┌───────────────┐           │
    │    │                         │   cable        │               │           │
    │    │  [4]  [5]  [6]    [14] │ ============== │   Plugs into  │           │
    │    │   │    │    │      │   │                │   car's OBD2  │           │
    │    │   │    │    │      │   │                │   port         │           │
    │    └───┼────┼────┼──────┼───┘                └───────────────┘           │
    │        │    │    │      │                                                 │
    │        │    │    │      └── Wire 8b (blue) ── CANL ── SN65HVD230        │
    │        │    │    └── Wire 8a (white) ── CANH ── SN65HVD230              │
    │        │    └── Wire 8d (black) ── GND rail                              │
    │        └── Wire 8c (black) ── GND rail                                   │
    │                                                                           │
    └─────────────────────────────────────────────────────────────────────────────┘
```

### Simplified Signal Flow Diagram

This shows how data flows through the system, from car to ESP32:

```
    ┌──────────┐         ┌─────────────┐         ┌──────────────────┐
    │          │  CANH   │             │  CRX    │                  │
    │   Car    ├────────>│  SN65HVD230 ├────────>│  ESP32-S3        │
    │   ECU    │  CANL   │  Transceiver│  (IO3)  │  (T-Beam)        │
    │          ├────────>│             │         │                  │
    │          │         │             │  CTX    │  Processes CAN   │
    │          │         │             │<────────┤  frames, logs    │
    │          │         │             │  (IO2)  │  to SD card      │
    └──────────┘         └──────┬──────┘         └────────┬─────────┘
                                │                         │
                             GND│(shared)              GND│(shared)
                                │                         │
                           ─────┴─────────────────────────┴─────
                                    Common Ground
```

### Wire Color Summary

| Wire | Color | From | To |
|------|-------|------|----|
| 1 | Red | T-Beam DC1 (3.3V) | Breadboard + rail |
| 2 | Black | T-Beam GND | Breadboard - rail |
| 3 | Red | Breadboard + rail | SN65HVD230 3V3 |
| 4 | Black | Breadboard - rail | SN65HVD230 GND |
| 5 | Yellow | T-Beam IO2 | SN65HVD230 CTX |
| 6 | Green | T-Beam IO3 | SN65HVD230 CRX |
| 7 | Black | SN65HVD230 Rs | Breadboard - rail |
| 8a | White | SN65HVD230 CANH | OBD2 Pin 6 |
| 8b | Blue | SN65HVD230 CANL | OBD2 Pin 14 |
| 8c | Black | OBD2 Pin 4 | Breadboard - rail |
| 8d | Black | OBD2 Pin 5 | Breadboard - rail |

---

## Appendix A: Shopping List

| Item | Quantity | Approximate Cost | Notes |
|------|----------|-----------------|-------|
| SN65HVD230 CAN Transceiver Module | 1 (buy 2 as spare) | $2-3 each | Search "SN65HVD230 breakout board" on Amazon/AliExpress. NOT the bare chip. |
| Half-size breadboard | 1 | $3-5 | Any standard breadboard works. |
| Jumper wire kit (M-M + M-F) | 1 kit | $5-8 | Get a multi-color assortment. At least 10 wires of each type. |
| OBD2 extension cable (M-F, 16 pin) | 1 | $5-10 | Search "OBD2 extension cable male to female." Make sure it has all 16 pins passed through. |
| USB-C cable | 1 | $5-10 | For connecting T-Beam to laptop. Must support data (not charge-only). |

**Total additional cost: approximately $20-35**

(You already have the LilyGo T-Beam Supreme.)

---

## Appendix B: OBD2 Full Pinout Reference

For reference, here is the complete OBD2 pinout. We only use pins 4, 5, 6, and 14.
The rest are included here so you know what NOT to touch.

```
              ╔═══════════════════════════════════════╗
             ╱  1    2    3    4    5    6    7    8   ╲
            ║   ●    ●    ●    ●    ●    ●    ●    ●   ║
            ║                                           ║
            ║   ●    ●    ●    ●    ●    ●    ●    ●   ║
             ╲  9   10   11   12   13   14   15   16  ╱
              ╚═══════════════════════════════════════╝
```

| Pin | Function | We Use? |
|-----|----------|---------|
| 1 | Vendor specific | No |
| 2 | J1850 Bus+ | No |
| 3 | Vendor specific | No |
| 4 | **Chassis Ground** | **YES** |
| 5 | **Signal Ground** | **YES** |
| 6 | **CAN High (ISO 15765-4)** | **YES** |
| 7 | ISO 9141-2 K-Line | No |
| 8 | Vendor specific | No |
| 9 | Vendor specific | No |
| 10 | J1850 Bus- | No |
| 11 | Vendor specific | No |
| 12 | Vendor specific | No |
| 13 | Vendor specific | No |
| 14 | **CAN Low (ISO 15765-4)** | **YES** |
| 15 | ISO 9141-2 L-Line | No |
| 16 | **Battery Positive (12V)** | **NEVER -- do NOT connect** |

---

## Appendix C: Glossary

| Term | Meaning |
|------|---------|
| **CAN bus** | Controller Area Network bus. A two-wire communication system used in cars. |
| **CANH / CANL** | CAN High and CAN Low. The two physical wires that carry CAN signals. |
| **Transceiver** | A device that can both transmit and receive. Converts between digital logic levels and CAN bus differential signals. |
| **Differential signal** | A signaling method that uses the voltage *difference* between two wires, making it resistant to electrical noise. |
| **OBD2** | On-Board Diagnostics version 2. A standardized 16-pin diagnostic connector in all modern cars. |
| **GPIO** | General Purpose Input/Output. A programmable pin on a microcontroller that can be used for various purposes. |
| **3V3** | 3.3 volts. The operating voltage of the ESP32 and the CAN transceiver. |
| **GND** | Ground. The zero-volt reference point shared by all components in a circuit. |
| **Breadboard** | A plastic board with interconnected holes for building circuits without soldering. |
| **Jumper wire** | A short flexible wire with connectors on each end for making breadboard connections. |
| **TWAI** | Two-Wire Automotive Interface. Espressif's name for the CAN controller built into the ESP32. |
| **baud rate / bps** | Bits per second. The speed of communication. CAN bus in cars typically runs at 500 kbps (500,000 bits per second). |
| **Termination resistor** | A 120-ohm resistor at each end of a CAN bus that prevents signal reflections. The car already has these. |
| **Silkscreen** | The white text and markings printed on a circuit board to label pins and components. |
| **Short circuit** | An accidental connection between two points that should not be connected, often causing damage. |
| **Continuity** | An electrical connection between two points. A multimeter's continuity mode tests for this. |
| **Brownout** | When the supply voltage drops too low, causing a microcontroller to reset. |
| **Stub** | Placeholder code that compiles but does not do anything useful yet. |
| **ECU** | Engine Control Unit. The car's main engine management computer. |
| **Gateway** | A module in some cars that filters CAN messages between different bus segments for security. |
