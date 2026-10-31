# HDP-001 servo selection: the owner's requirements, the options, the candidate log

- Started 2026-10-31. Supporting material for [HDP-001](HDP-001.md), whose §1.4 is the per-axis
  specification and whose §8 is the side-by-side comparison.
- **Use this file when a new joint or servo candidate comes in:**
  - check it against §2 and §3 here;
  - estimate its accuracy with §5;
  - place it among the options in §4;
  - log it in §7.

## 1. The owner's requirements and rulings for selection

| Date | Ruling | Effect |
|---|---|---|
| 2026-10-31 | Candidates in order: the plan's list first, then the MZ17-mini, then other suppliers | HDP-001 §8.1, then §8.5 |
| 2026-10-31 | **Price matters.** The eRob90 costs almost twice the MZ17, and the owner would accept slightly lower accuracy if the saving is worth it | precision rows can be traded for price (§3), judged on the estimated total (§5), not row by row |
| 2026-10-31 | Plugs are fitted after the cables are routed, so the bore is not sized by connectors | HDP-001 §1.4 F5: must ≥ 15 mm usable |
| 2026-10-31 | A datasheet for the MZ17 will follow; the sheet in hand is preliminary | HDP-001 §8.6 |

**Open: what "slightly lower" means as a number.** Reference points:
- the spec, 0.3 MOA = 18″ (13 cm at 1500 m);
- ≈ 0.4 MOA (17–19 cm), which still keeps the beam on the IPSC target at 1500 m (half-width ±32″
  = 0.53 MOA), with less margin left for beam divergence.

The owner's floor goes here once given.

## 2. Hard gates: a candidate that misses one is out, whatever the price

This split between §2 and §3 is a proposal, for the owner to confirm.

| Gate | HDP-001 row | Why it is not for sale |
|---|---|---|
| Gearing without tooth clearance: strain-wave or precision cycloidal (a catalogue's "背隙 15″" on a strain-wave joint is read as lost motion, A9) | A8 | planetary backlash is arc-minutes (§5) |
| Torsional stiffness at least the must: yaw 1.0×10⁴, pitch 0.8×10⁴ N·m/rad | B1 | no loop setting buys stiffness back (§2) |
| A documented CAN protocol that carries position finer than 1″ | G1 | a coarse bus step quantises the aim itself (MyActuator's 0.01° = 36″) |
| A torque limit that can be set in the drive | D4 | at full peak, these joints crush a hand and twist a tripod |
| A brake on pitch whenever pitch can be out of balance at shutdown | D5 | owner ruling: never de-energise an unbalanced load |
| Telemetry, and an output zero kept in the joint | G6, G7 | the fault, hold and degrade ruling; no homing |
| Usable yaw bore ≥ 15 mm (or a way around it: §4 option 3) | F5 | the slip ring's cables have to pass |
| 24–48 V, and torque and loads at least the must | G3, D1–D2, B3–B4 | |

## 3. Tradeable for price, judged on the total of §5

| Row | What is given up |
|---|---|
| A4: output encoder 19 bit instead of 20 | about ±1.2″ more hold error per axis |
| G2 on pitch: the loop closes on the motor encoder | pitch's backlash and wind-up reach the payload. Worth about +6–8″ on pitch, less with the preload of HDP-001 §6.6 |
| A2, A3: vendor repeatability and accuracy | the camera loop removes both once the joint has arrived (HDP-001 §1.3). They matter only for a first shot taken before the camera corrects |
| G4: IP rating | a cover gives it |
| E6: loop rates not stated | phase 1 measures what they deliver |

## 4. The options

The estimates come from §5 and are not measurements. Cost is relative, with the MZ17 = 1: the owner
puts the eRob90 at about 2.

| Option | Yaw | Pitch | Total, loop on the output encoder | Total, loop on the motor encoder | Cost | Conditions |
|---|---|---|---|---|---|---|
| 1 | eRob90 HS | eRob90 HS + brake | ≈ 14.6″ ≈ 0.24 MOA | — (the eRob closes on its output) | ≈ 4 | the plan's §8.3; ZeroErr does not guarantee its own figures |
| 2 | eRob90 HS | **MZ17**, 101:1 | ≈ 15″ ≈ 0.25 MOA | ≈ 18″ ≈ 0.30 MOA; ≈ 0.26 with the preload | ≈ 3 | the MZ17's brake, CAN protocol and position unit (§2) |
| 3 | MZ17 | MZ17 | ≈ 15″ ≈ 0.25 MOA, but 8 m/s gusts throw yaw ≈ 9″ before the loop recovers | ≈ 24″ ≈ 0.40 MOA; ≈ 0.36 with the preload | ≈ 2 | yaw stiffness exactly on the must, no margin. The Ø12 bore needs thin cables (~10 mm), or the yaw joint mounted inverted so that CAN stays off the slip ring |
| 4 | MZ20 | MZ17 | ≈ 15″ ≈ 0.25 MOA | ≈ 22″ ≈ 0.36 MOA; ≈ 0.32 with the preload | ≈ 2–3 | the MZ20's bore, encoder and price are not on the sheet |
| 5 | SHG-20 build | SHG-20 build | ≈ 14″ ≈ 0.24 MOA | — | not priced (the drive alone is ≈ USD 770 per axis) | two axes of mechanical design. Fallback if phase 1 fails (HDP-001 §4) |

**What the table says:**
- **Once the joint closes its loop on the output encoder, joint precision stops deciding the
  total.** About 14″ of the 18″ belongs to calibration, heat shimmer and the base (§5). With the
  mechanics below about 5″ per axis, a better joint buys about 0.01 MOA. Paying twice for the eRob90
  on pitch is then hard to justify.
- **The deciding unknown is G2.** Does the MZ17's position loop close on its output encoder? If it
  does not, option 2 still lands at about 0.30 MOA, which is the spec. Option 3 lands at about
  0.4 MOA.
- **Yaw is where the eRob90 earns its price:**
  - its bore clears 15 mm;
  - its stiffness (1.6×10⁴) has margin over the must;
  - its loop on the output encoder is confirmed (ZeroErr's CANopen manual V1.9).

## 5. Estimating a candidate's accuracy

Per axis, the error while holding in an 8 m/s wind.

**Loop on the output encoder:**

  e ≈ (±1 count of the output encoder, plus its noise) + (a gust's T / K1, removed within E1)

- ±1 count is 2.5″ at 19 bit and 1.2″ at 20 bit.
- Typical result: 3–4″ for a size 17 joint at 19 bit, 2–3″ for size 20 at 20 bit.

**Loop on the motor encoder:**

  e ≈ lost motion / 2 + T / K1 + the gear's error over a small correction

- The gear's error over a small correction is a few arcseconds for a 1–1.5′ gear.
- Typical result: 10–12″ on pitch and 15–17″ on yaw, for a size 17 joint with 15″ of backlash.
- With the pitch preload of HDP-001 §6.6, the backlash term on pitch drops to about zero (pitch
  ≈ 4–6″).

**Constants:**
- T at 8 m/s (HDP-001 P4): yaw 0.42 N·m, pitch 0.14 N·m.
- K1 from the datasheet, or from the gear size class at the ratio ordered. At ≥ 80:1: size 17 is
  1.0×10⁴, size 20 is 1.6×10⁴. At 50:1 it is one step softer.

**Total:**

  total ≈ √(e_yaw² + e_pitch² + 14²)

The 14″ is what the plan's split (18″ in all, 8″ per axis, combined as a root-sum-square) leaves for
calibration, heat shimmer and the base. It is an allowance, not a measurement. If those turn out
smaller, joint precision matters more, and this section should be redone with the measured value
(phase 3 and phase 4).

## 6. Evaluating a new candidate

1. Get the datasheet, and fill a column like HDP-001 §8.1 for it (the rows of §1.4).
2. Check the hard gates (§2). A miss ends the evaluation; log it in §7.
3. Ask the vendor the questions datasheets leave out:
   - which encoder closes the position loop, and at what rate (G2, E6);
   - the bus protocol and its position unit (G1);
   - the output encoder's bits and repeatability (A4, A6);
   - K1 at the ratio ordered (B1);
   - lost motion, and how "背隙" is measured (A9);
   - whether the bore's wall is static, turns with the output, or spins with the motor (F5);
   - the brake and its holding torque (D5);
   - the IP rating and the operating temperature (G4, G5);
   - the price.
4. Estimate its accuracy with §5, in both loop cases if G2 is not yet answered.
5. Place it in the options (§4), at its cost relative to the MZ17.
6. Phase 1 (HDP-001 §7) measures the chosen joint on the bench. 8″ is 1–2 mm on a target 50 m away.
   The bench decides, not the datasheet.

## 7. Candidate log

| Candidate | Axis | Status (2026-10-31) | Next step |
|---|---|---|---|
| ZeroErr eRob90 HS, CANopen | yaw (and pitch in option 1) | **lead for yaw** | §8.4 questions to ZeroErr; quote the HS / CANopen / brake variants |
| MZ17, 101:1 (Tmall MZ17-mini) | pitch (both axes in option 3) | **value candidate for pitch**, preliminary sheet only | datasheet; the §8.4 questions, G2 first; brake? |
| MZ20 | yaw (option 4) | to ask | bore, encoder, price |
| ZeroErr eRob80 HS | pitch | alternative to the MZ17, at about the eRob90's price | none unless the MZ17 fails a gate |
| SHG-20 build (HDP-001 §4) | both | fallback if phase 1 fails | — |
| Spinea TwinSpin build | pitch only | not pursued: no bore; the standard lost motion misses A9 | — |
| MyActuator RH | — | rejected: 17-bit output encoder, 36″ CAN position step | — |
| Leaderdrive 绿的谐波 KAH / KAT | either | to ask: is the encoder on the output? | HDP-001 §8.5 |
| Leadshine 雷赛 RJ1 / RJH | either | to ask: encoder bits | HDP-001 §8.5 |
| HD SHA-20 integrated, EYOU PH17/20, Honpine HPJM-RE, Infranor RJM, Synapticon, Rozum and others | — | out (HDP-001 §8.5) | — |

## 8. Open for the owner

- The accuracy floor that "slightly lower" means (§1).
- Whether the gate/tradeable split of §2–§3 is right.
- The pitch preload (HDP-001 §6.6), which needs the pitch brake.
- Whether the yaw joint may be mounted inverted (option 3 and the bore): its body would ride on the
  yaw stage, keeping CAN off the slip ring.
