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
| 2026-11-10 | The datasheets arrived, for the MZ17-mini (Juxie R68 Lite), which the owner reads as ticking more boxes than the MZ17 | HDP-001 §8.8; transcribed in [`juxie-r68-datasheets.md`](juxie-r68-datasheets.md) |
| 2026-11-10 | **The MZ17 series is out.** Its position covers one turn, so it cannot drive a continuously turning yaw | options 2–4 of §4 struck; the price trade moves to the suppliers still to ask (§7) |
| 2026-11-10 | Joint life is not a concern: the mount needs accuracy, not torque, so the joints run far below their rating | a vendor's rated life (e.g. 5000 h) is not a gate |
| 2026-11-10 | Screen cycloidal reducers and servo combos too, Chinese and China-made first | HDP-001 §8.9; candidates in §7 |
| 2026-11-10 | **Price.** The eRob90 is about 8,000 CNY, a bit out of range. The owner accepts lower accuracy, up to about 0.3 MOA on both axes. Wants a side-by-side comparison of every option, with none eliminated yet | §9 |
| 2026-11-10 | **Yaw turns continuously** (through the slip ring): the yaw joint must command and report position across more than one turn, or across ±180° without a fault | new gate in §2 |

**The floor (owner, 2026-11-10): 0.3 MOA = 18″ on each axis, not in total.**
- Each axis's joint may contribute up to about 15″, after its share of the 14″ allowance for
  calibration, heat shimmer and the base (HDP-001 §1.3).
- The combined error of both axes may reach about 25″ (0.42 MOA). That still keeps the beam on the
  IPSC target at 1500 m (half-width ±32″ = 0.53 MOA), with less margin for beam divergence.
- HDP-001 §1.1, §1.3 and rows A1, A4, C3, E1, E2 and E4 were rewritten to match. B1's stiffness and
  E4's peak are unchanged.

## 2. Hard gates: a candidate that misses one is out, whatever the price

This split between §2 and §3 is a proposal, for the owner to confirm.

| Gate | HDP-001 row | Why it is not for sale |
|---|---|---|
| Gearing without tooth clearance: strain-wave or precision cycloidal (a catalogue's "背隙 15″" on a strain-wave joint is read as lost motion, A9) | A8 | planetary backlash is arc-minutes (§5) |
| Torsional stiffness at least the must: yaw 1.0×10⁴, pitch 0.8×10⁴ N·m/rad | B1 | no loop setting buys stiffness back (§2) |
| A documented CAN protocol whose position step keeps the axis within its 15″ share (§5): a step of up to about 20″ just fits; finer than 2.5″ is recommended (until 2026-11-10: finer than 1″) | G1 | a coarse bus step quantises the aim itself. Juxie's 19.8″ step gives 10–14″ and fits; MyActuator's 0.01° (36″) gives 17–20″ and does not |
| A torque limit that can be set in the drive | D4 | at full peak, these joints crush a hand and twist a tripod |
| A brake on pitch whenever pitch can be out of balance at shutdown | D5 | owner ruling: never de-energise an unbalanced load |
| Telemetry, and an output zero kept in the joint | G6, G7 | the fault, hold and degrade ruling; no homing |
| Usable yaw bore ≥ 15 mm (or a way around it: §4 option 3) | F5 | the slip ring's cables have to pass |
| 24–48 V, and torque and loads at least the must | G3, D1–D2, B3–B4 | |
| Yaw only: position commanded and reported across more than one turn, or across ±180° without a limit trip or a rejected step | G1, G2 | yaw turns continuously through the slip ring (owner, 2026-11-10); a single-turn drive leaves no way round its seam |

## 3. Tradeable for price, judged per axis on §5 (each axis ≤ 18″, the joint ≤ about 15″)

| Row | What is given up |
|---|---|
| A4: output encoder 20 bit instead of 19 (19 is now the must) | 20 bit buys about ±1.2″ less hold error per axis |
| G2 on pitch: the loop closes on the motor encoder | pitch's backlash and wind-up reach the payload. Worth about +6–8″ on pitch, less with the preload of HDP-001 §6.6 |
| A2, A3: vendor repeatability and accuracy | the camera loop removes both once the joint has arrived (HDP-001 §1.3). They matter only for a first shot taken before the camera corrects |
| G4: IP rating | a cover gives it |
| E6: loop rates not stated | phase 1 measures what they deliver |

## 4. The options

The estimates come from §5 and are not measurements. Cost is relative, with the MZ17 = 1: the owner
puts the eRob90 at about 2. The MZ17 stays the cost yardstick although it is out.

**Options 2–4 are out (owner, 2026-11-10):** the MZ17 series covers one turn, so it fails the yaw
gate of §2, and on pitch its 19.8″ protocol step and missing brake fail G1 and D5 (HDP-001 §8.8).
After the per-axis floor of the same day (§1), the step fits G1. So an MZ17-mini on pitch fails only
D5, and option 2 is back as a pitch option in §9.
They are kept below as the record of what the price trade was worth. A cheaper pitch candidate from
§7's "to ask" rows would take option 2's place.

| Option | Yaw | Pitch | Total, loop on the output encoder | Total, loop on the motor encoder | Cost | Conditions |
|---|---|---|---|---|---|---|
| 1 | eRob90 HS | eRob90 HS + brake | ≈ 14.6″ ≈ 0.24 MOA | — (the eRob closes on its output) | ≈ 4 | the plan's §8.3; ZeroErr does not guarantee its own figures |
| 2 (out) | eRob90 HS | **MZ17**, 101:1 | ≈ 15″ ≈ 0.25 MOA | ≈ 18″ ≈ 0.30 MOA; ≈ 0.26 with the preload | ≈ 3 | the MZ17's brake, CAN protocol and position unit (§2) |
| 3 (out) | MZ17 | MZ17 | ≈ 15″ ≈ 0.25 MOA, but 8 m/s gusts throw yaw ≈ 9″ before the loop recovers | ≈ 24″ ≈ 0.40 MOA; ≈ 0.36 with the preload | ≈ 2 | yaw stiffness exactly on the must, no margin. The Ø12 bore needs thin cables (~10 mm), or the yaw joint mounted inverted so that CAN stays off the slip ring |
| 4 (out) | MZ20 | MZ17 | ≈ 15″ ≈ 0.25 MOA | ≈ 22″ ≈ 0.36 MOA; ≈ 0.32 with the preload | ≈ 2–3 | the MZ20's bore, encoder and price are not on the sheet |
| 5 | SHG-20 build | SHG-20 build | ≈ 14″ ≈ 0.24 MOA | — | not priced (the drive alone is ≈ USD 770 per axis) | two axes of mechanical design. Fallback if phase 1 fails (HDP-001 §4) |

**What the table says:**
- **Once the joint closes its loop on the output encoder, joint precision stops deciding the
  total.** About 14″ of the 18″ belongs to calibration, heat shimmer and the base (§5). With the
  mechanics below about 5″ per axis, a better joint buys about 0.01 MOA. Paying twice for the eRob90
  on pitch is then hard to justify.
- **For a cheaper pitch joint, the deciding rows are G2 and G1's position step.** A joint that
  closes on its output encoder and carries position finer than 1″ lands within about 0.01 MOA of the
  eRob90. One that closes on the motor encoder lands at about 0.30 MOA, which is the spec.
- **Yaw is where the eRob90 earns its price:**
  - its bore clears 15 mm;
  - its stiffness (1.6×10⁴) has margin over the must;
  - its loop on the output encoder is confirmed (ZeroErr's CANopen manual V1.9);
  - **still to confirm:** that its drive runs past ±180° (the yaw gate of §2, HDP-001 §8.4).

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
| ZeroErr eRob90 HS, CANopen | yaw (and pitch in option 1) | **lead for yaw** | §8.4 questions to ZeroErr, including the yaw gate of §2 (position past ±180°); quote the HS / CANopen / brake variants |
| MZ17, 101:1 (= Juxie R68 DNN) | pitch | out with the series (2026-11-10) | — |
| MZ17-mini, 101:1 (= Juxie R68 Lite), 2026-11-10 | both axes, or pitch | **out for yaw (owner, 2026-11-10):** position covers one turn, so no continuous yaw. **Pitch: possible.** Under the per-axis floor (§1) its 19.8″ step fits G1 (10–14″). It has no brake (D5). Clears A4, F5 and G2 | pitch: ask Juxie for a brake, or confirm that pitch is balanced at every shutdown; yaw: reopen only if Juxie documents multi-turn position |
| MZ20 | yaw (option 4) | out with the series (2026-11-10): it was the yaw size, and the series covers one turn | — |
| ZeroErr eRob80 HS | pitch | alternative to the eRob90 on pitch, at about its price | only if the payload's mass (phase 0) makes the eRob90's 0.5 kg matter |
| SHG-20 build (HDP-001 §4) | both | fallback if phase 1 fails | — |
| Spinea TwinSpin build | pitch only | not pursued: no bore; the standard lost motion misses A9 | — |
| **PHYBOT 动易 PhyArc** (integrated cycloidal), 2026-11-10 | either | **to ask**: the only integrated cycloidal joint found. Own CAN FD protocol; encoder bits, loop encoder, bore and brake not published (HDP-001 §8.9) | the owner asks PHYBOT §8.9's five questions; closes on the answers or a bench unit |
| Cycloidal reducers for a build: 环动 SHPR-H, 西格 CPB-072H / 086H, 2026-11-10 | either | not pursued: no published lost motion or bore, and a build costs what the SHG-20 build does, with worse lost motion (HDP-001 §8.9) | reopen only if the SHG-20 build becomes the path (phase 1 fails) |
| 海尚 HSOAR SD-BR85C; industrial RV-10C class (双环, 秦川, 振康, 中大力德, 智同, 飞马, Nabtesco); Sumitomo Fine Cyclo | — | out: lost motion ≤ 2′ (A9), or Ø147 mm and 4.6 kg for the gear alone (F2, F4), or no small hollow size (HDP-001 §8.9) | — |
| MyActuator RH | — | rejected: 17-bit output encoder, 36″ CAN position step | — |
| Leaderdrive 绿的谐波 KAH / KAT | either | to ask: is the encoder on the output? | HDP-001 §8.5 |
| Leadshine 雷赛 RJ1 / RJH | either | to ask: encoder bits | HDP-001 §8.5 |
| HD SHA-20 integrated, EYOU PH17/20, Honpine HPJM-RE, Infranor RJM, Synapticon, Rozum and others | — | out (HDP-001 §8.5) | — |

## 8. Open for the owner

- Whether the gate/tradeable split of §2–§3 is right.
- The pitch preload (HDP-001 §6.6), which needs the pitch brake.

## 9. Price and accuracy, side by side (2026-11-10)

The owner asked for a CNY price range per option, with nothing eliminated, against an accuracy of
about 0.3 MOA on both axes. Gates that a candidate fails are shown as conditions, not as verdicts.

### 9.1 How the numbers were made

**Prices are ranges per joint, not quotes.**
- Only one figure comes from the owner: the eRob90, at about 8,000 CNY.
- For other Chinese brands, overseas retail prices were scaled to a China price. The owner's eRob90
  figure gives the factor: overseas retail is USD 1,610–2,327 (11–16.5k CNY), so a China price is
  about 0.5–0.7× overseas retail.
- Exchange rates assumed: 1 USD ≈ 7.1 CNY, 1 EUR ≈ 7.7 CNY.
- "Class" means an order-of-magnitude figure with no source for that part. Treat it as a placeholder
  until a quote comes in.

**Accuracy follows §5.**
- *e* is the joint's own hold error per axis, in an 8 m/s wind.
- **Per axis** is that error plus half of §5's 14″ allowance for calibration, heat shimmer and the
  base: √(e² + 9.9²).
- **Total** is §5's combination of both axes: √(e_yaw² + e_pitch² + 14²).

**Two readings of the owner's "0.3 MOA on both axes":**
- (a) **Each axis ≤ 0.3 MOA (18″). The owner's ruling, 2026-11-10 (§1).** This allows e up to about 15″ per axis, and a total of up to
  about 0.42 MOA.
- (b) **The total ≤ 0.3 MOA,** HDP-001's spec until then, kept for reference. This allows e up to about 7.5″ per axis.
- Most of the cheap joints pass (a) and fail (b).

### 9.2 Per joint

| # | Joint | Gear | CNY per joint | Basis of the price | Axes | e per axis | Per axis | Decides it, or fails as documented |
|---|---|---|---|---|---|---|---|---|
| 1 | ZeroErr eRob90 HS, CANopen | strain-wave 20 | 7–10k | owner ≈ 8k; overseas USD 1,610–2,327 | both | 2–3″ | 0.17 | yaw: does position run past ±180° (§2)? |
| 2 | ZeroErr eRob80 HS | strain-wave 17 | 5–8k | class: below the eRob90 | pitch; yaw on B1's line | 3–4″ (yaw: gusts ≈ 9″ transient) | 0.18 | — |
| 3 | ZeroErr eRob70 | strain-wave 14 | 4–6k | class | pitch | 4–6″ | 0.19 | yaw fails B1 (gusts ≈ 18″); 19-bit encoder (A4) |
| 4 | Juxie R68 Lite (MZ17-mini) | strain-wave 17 | 1.5–4k | owner: eRob ≈ 2× MZ17; Juxie's Lite line starts at 699 CNY (R48-Lite) | both | 10–14″ (19.8″ bus step) | 0.23–0.29 | position covers one turn (yaw); no brake (D5) |
| 5 | Juxie R68 DNN (MZ17) | strain-wave 17 | 1.5–4k | as #4 | pitch (Ø12 bore) | 10–14″ | 0.23–0.29 | as #4; 19-bit encoder |
| 6 | Juxie R83 (MZ20) | strain-wave 20 | 3–6k | class, from #4 | yaw | 10–14″ (if it has the R68's protocol) | 0.23–0.29 | as #4 |
| 7 | EYOU 意优 PH17 / PH20, CANopen | strain-wave 17 / 20 | 3–5k | overseas USD 700–1,300 | both (bore 11–18 mm: sources disagree) | 3–4″ if the loop is on the output encoder, 10–17″ if not | 0.18–0.33 | which encoder closes the loop; 19-bit (A4) |
| 8 | MyActuator RH-20 | strain-wave 20 | 4–7k | overseas USD 930–1,700 | both | 17–20″ (36″ bus step, 17-bit) | 0.33–0.37 | the bus step (G1) |
| 9 | Leaderdrive 绿的 KAH + KDE drive | strain-wave | 5–10k | class; the gear alone lists at 1.1–3.4k | both | 2–4″ (output loop) or 10–17″ | 0.17–0.33 | is the encoder on the output? |
| 10 | Leadshine 雷赛 RJ1 / RJH | strain-wave | 3–6k | class; "economy" line, 35% below imports (the maker) | both | 10–20″ (±36″ repeatability) | 0.23–0.37 | encoder bits |
| 11 | Honpine 宏品 HPJM-RE | strain-wave | 8–20k (?) | overseas listings USD 3.8–4.7k, size unclear | both | 10–14″ (16-bit on one page) | 0.23–0.29 | encoder bits; CANopen only on the single-encoder version |
| 12 | Harmonic Drive SHA-20, integrated drive | strain-wave 20 | 20–40k | class; quote only | both (bore ≈ 17 mm) | 10–14″ (16-bit output encoder) | 0.23–0.29 | price |
| 13 | PHYBOT 动易 PhyArc 68 / 78 | cycloidal 25:1 | 6–15k | PhyArc 78 is USD 2,050 overseas | both, if hollow | 3–5″ (output loop) or ≈ 30″ (motor loop, 1′ backlash) | 0.18–0.53 | HDP-001 §8.9's questions |
| 14 | SHG-20 build (HD SHG-20 + Kollmorgen KBM-17 + Renishaw RESA30 + Ingenia Everest) | strain-wave 20 | 25–45k | class: gear USD 1–1.8k, encoder 1–2k, drive 645–800, plus machining | both | 1–2″ | 0.16 | two axes of mechanical design |
| 15 | China-made strain-wave build (Leaderdrive LHSG-20 + 85 mm frameless motor + RLS AksIM-2 + a CANopen dual-loop drive) | strain-wave 20 | 6–15k | gear 1.1–3.4k; motor 1–3k (CubeMars RI85 ≈ 990); AksIM-2 ≈ EUR 221 (17-bit; 20-bit on request), so 1.5–3k; drive 1.5–5k; housing 1.5–4k | both | 3–4″ | 0.18 | the design work of #14; the drive's dual loop |
| 16 | Spinea TwinSpin TS 70 H build | cycloidal | 20–35k | class | pitch (13 mm bore) | 2–3″ | 0.17 | lost motion < 1.5′ (A9): reversal hunting |
| 17 | Chinese cycloidal build (环动 SHPR-H, 西格 CPB, 海尚 SD-BR85C + #15's parts) | cycloidal | 7–15k | class: reducer 2–5k (no price found) | both, if the bore allows | 3–5″ | 0.18 | lost motion 1–2′ (A9); no data |
| 18 | Nabtesco RV-10C build | RV | 20–35k | class | yaw only (Ø147 mm, 4.6 kg for the gear alone) | 2–3″ | 0.17 | size and mass (F2, F4) |

### 9.3 Pairs worth comparing

| Yaw + pitch | CNY for both axes | Per axis, yaw / pitch | Total | Reading (a): each axis ≤ 0.3 | Reading (b): total ≤ 0.3 |
|---|---|---|---|---|---|
| eRob90 + eRob90 | 14–20k | 0.17 / 0.17 | 0.24 MOA | ✓ | ✓ |
| eRob90 + eRob80 | 12–18k | 0.17 / 0.18 | 0.25 | ✓ | ✓ |
| eRob90 + EYOU PH17 | 10–15k | 0.17 / 0.18–0.33 | 0.24–0.37 | ✓ if EYOU's loop is on the output | ✓ only in that case |
| eRob90 + R68 Lite | 8.5–14k | 0.17 / 0.23–0.29 | 0.29–0.33 | ✓ | on the line |
| EYOU + EYOU | 6–10k | 0.18–0.33 each | 0.25–0.46 | ✓ if the loop is on the output | ✓ only in that case |
| China-made build ×2 (#15) | 12–30k | 0.18 each | 0.25 | ✓ | ✓, after two axes of design |
| R68 Lite + R68 Lite | 3–8k | 0.23–0.29 each | 0.33–0.40 | ✓ on accuracy; yaw cannot turn past one turn | ✗ |
| MyActuator RH-20 ×2 | 8–14k | 0.33–0.37 each | 0.46–0.53 | ✗ | ✗ |

### 9.4 What the comparison says

- **Below the eRob's price, the one row that decides a joint is whether its position loop closes on
  the output encoder** (G2), and with what bus step (G1). Joints that close on the output encoder sit
  at about 0.18 MOA per axis whatever they cost. The rest sit at 0.23–0.37.
- **EYOU's PH17 / PH20 is the cheapest unknown that could match the eRob**, at about half its price.
  One answer decides it: which encoder closes its position loop.
- **On reading (a), the Juxie R68 Lite passes on accuracy at the lowest price.** It still fails on
  yaw's one-turn range and has no brake. As pitch under an eRob90 yaw, it is the cheapest pair that
  passes (a).
- **Building our own joint is not cheaper once the design work is counted.** #15's parts come to
  roughly the eRob90's price, before the design and machining of two axes.

**Price sources, 2026-11-10:**
- eRob90 overseas: [aifitlab](https://aifitlab.com/products/zeroerr-erob-90-t-motor), [rcdrone](https://rcdrone.top/products/zeroerr-erob-90-t-motor)
- MyActuator RH-20: [RobotShop](https://www.robotshop.com/products/myactuator-mt-rh-20ethercatcan-harmonic-joint-motor-robot-joint-motor-large-hollow-design-high-precision-high-torque-harmonic-reducer), [OZ Robotics](https://ozrobotics.com/shop/rh-series-standard-type-motor-with-integrated-design-and-vibration-resistance-mt-rh-20-100-b/)
- EYOU: [RobotShop PH17B](https://www.robotshop.com/products/eyou-robotics-canopen-harmonic-actuator-with-brake-134-nm-instantaneous-30-rpm-ph17b-101-c), [aifitlab PH20](https://aifitlab.com/products/eyou-ph20-motor), [sicubeshop](https://sicubeshop.com/products/ph-series-mechanical-arm-hollow-harmonic-servo-motor-robot-joint-module)
- Juxie R48-Lite at 699 CNY: [OFweek](https://mp.ofweek.com/robot/a256714842617)
- PhyArc 78 at USD 2,050: [Humanoid.guide](https://humanoid.guide/cycloidal-reducers-humanoid-joints/)
- Leaderdrive LCSG / LHSG-20 listings: [gys.cn](https://m.gys.cn/jiansuji/5138751456.html)
- CubeMars RI85: [DirectIndustry](https://www.directindustry-china.cn/zhizaoshang-gongye/48v-li-ju-dian-ji-222855.html)
- RLS AksIM-2: [MAB Robotics](https://www.mabrobotics.pl/product-page/rls-aksim-2)
- Ingenia Everest: [Novanta](https://novanta.com/robotics-automation/product/everest/)
- Honpine: [RobotShop](https://www.robotshop.com/products/honpine-hollow-shaft-integrated-harmonic-drive-servo-motor-rotary-actuators)
