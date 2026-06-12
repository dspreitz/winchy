# Physics of the Glider Winch Launch

Working knowledge base for Winchy: what happens in each phase of a winch
launch, the forces involved, typical numbers, and what it all means for the
rope unit's measurements and the throttle advice shown to the winch driver.
Sources at the end.

## The system

Three coupled actors: the **winch** (300–500 HP, drum winding 1000–1500 m of
Dyneema rope), the **rope** (with parachute, weak link, and — in our case —
the instrumented weak link of the rope unit), and the **glider**. Control is
split: on a classic speed-controlled (SC) winch the driver's throttle
determines glider airspeed and the pilot can only trade pitch for climb
steepness; on a modern automatic-tension-controlled (ATC) winch the machine
holds rope tension and the pilot controls airspeed with pitch. **Winchy
effectively retrofits the sensing half of an ATC winch onto an SC winch**:
it measures tension at the glider end and tells the driver what an ATC
controller would do.

Two key control variables, only one of which can be commanded at a time:

- **Rope tension T** — what the winch (and Winchy's advice) controls.
- **Airspeed v** — what the pilot controls with pitch once tension is steady.

The fundamental coupling: raising the nose increases load and (on an SC
winch) slows the engine; more throttle raises both speed and tension. No two
SC launches are alike, which is exactly the problem Winchy addresses.

## Phase-by-phase

A complete launch takes ~30–45 s from standstill to release at roughly
**half the rope length** in height (more with headwind), e.g. ~400–600 m
height from 1000–1200 m of rope.

### 0. Take up slack

Drum turns slowly until the rope is straight and just taut. Tension: tens of
kgf, well below glider weight. Ends with the "all out" call.

**Winchy signals:** tension rises from ~0 to a small steady value. Good
moment to auto-tare drift, timestamp the launch, and reset the per-tow log.

### 1. Ground roll (≈ 0–3 s)

The throttle is advanced smoothly over ~2–4 s. Force balance on the glider
(rope roughly horizontal, hook near the CG):

```
m·a = T − D_aero − μ·N        N = m·g − L   (wheel reaction)
```

Early in the roll, aerodynamic drag D and lift L are tiny, so **T ≈ m·a**:
this 2–3 second window is where the glider mass estimate works — fit
T against measured longitudinal acceleration before lift-off, while μ·N and
D are small corrections.

Typical values: acceleration up to ~1 g (the weak link caps the theoretical
maximum near 1.5 g); lift-off at 35–45 kt after 40–60 m of ground run.
German Aero Club winch requirements: aileron control within 15 m, take-off
speed within 50 m, rope speed capability ≥ 1.2× lift-off speed. Brisk
acceleration is a *safety feature*: it minimizes the window in which a wing
drop can become a ground loop, and it banks speed for a possible low rope
break.

**Winchy signals:** sharp tension rise plus sustained longitudinal
acceleration. This phase has the highest demand on force sample rate.
**Advice logic:** report achieved acceleration; flag sluggish (< ~0.5 g) or
excessive ramps. Peak power demand of the whole launch comes at the *end*
of this phase: P = T·v_rope is largest when rope speed is greatest.

### 2. Rotation (≈ 3–8 s, the critical phase)

The pilot pitches from level to the ~40–45° climb attitude. This is the
most dangerous part of the launch:

- The wing is loaded by the rope pull *and* by curving the flight path
  upward (vertical speed goes from 0 to 45+ kt), so the **loaded stall
  speed rises significantly** exactly when airspeed is still building.
- A stall here with any asymmetric control input can snap-roll — the
  classic fatal winch accident.
- Guidance: don't begin rotation below ~50 kt; rotation rate ≤ 10°/s
  unless speed is abundant; airspeed must increase monotonically through
  the rotation.

For the winch driver this means: **smooth, constant tension — no surging.**
Automotive-drivetrain winches show tension oscillations up to 75% peak-to-
peak right here, which is the main thing the operator must be helped to
avoid.

**Winchy signals:** rope angle (from IMU) climbing from ~0–10° toward
30–40°; tension still high; barometric altitude starting to rise.
**Advice logic:** hold power steady; warn on tension oscillation or on
tension exceeding a weak-link margin (e.g. > 80% of rated break load).

### 3. Full climb (≈ 8–30 s)

Quasi-steady climb. With tension factor k = T/W (W = m·g, glider weight):

- k ≈ 1.0 supports a peak climb angle near **45°**; lower tension →
  shallower climb, higher tension → steeper. Practical ATC setpoints run
  k ≈ 0.7–1.5 depending on glider and pilot preference.
- The wing carries `L ≈ W·cosγ + T·sin(θ_rope+γ)`-type loading — i.e. well
  above 1 g; total wing load approaches W + T near the top. This is why
  open airbrakes (lift shifted to the tips) endanger the spar.

Crucial kinematic fact for throttle advice: the rope winds in at
`v_rope = v_glider · cos(β)` where β is the angle between the glider's
flight path and the rope. β grows continuously during the climb, so **to
keep airspeed constant the drum must slow down by roughly two-thirds over
the launch**. A constant-throttle SC winch therefore makes the glider
*faster and faster* as the launch progresses — the driver must taper power
all the way up, and most of Winchy's value in this phase is telling him by
how much.

Wind matters: the *less* headwind, the *more* power is needed; a headwind
gradient with height effectively adds airspeed for free and requires
earlier power reduction.

**Winchy signals:** rope angle steadily increasing 30° → 60°+; pressure
altitude rate ≈ v·sinγ; tension ideally constant at the target factor.
**Advice logic:** drive toward constant tension k·W using the measured
mass from phase 1; anticipate the taper rather than reacting late.

### 4. Top of launch / round-over (last ~5 s)

As the glider comes near overhead, climb flattens, β → 90°, rope speed →
small, and any retained power converts into overspeed. The driver
**reduces power early and decisively**; experienced drivers cut power well
before the glider passes overhead. With power cut, rope sag increases, the
rope-to-glider angle reaches ~70–75°, and the Tost CG hook **back-releases
automatically** (the pilot pulls the release anyway). Good technique means
near-zero tension at the moment of release.

**Winchy signals:** rope angle > ~60°, altitude rate decaying, tension
falling.
**Advice logic:** "reduce / cut power" indication; confirm release by
tension dropping to zero and staying there.

### 5. After release / failure modes

Release at ~45–50% of rope length AGL; the parachute brings the rope down
while the drum winds it in. The launch record should close with a tow
summary (max tension, release height, mass estimate, duration).

Failure physics worth knowing because Winchy sees them first:

- **Rope/weak-link break:** tension steps to zero instantly. At 45° nose-up
  the glider loses ~12 kt/s — the pilot must push over to ~0 g within
  fractions of a second. A break detected by the rope unit (tension step
  while angle < 60°) is worth an immediate, prominent indication to the
  winch driver ("Abort").
- **Winch power fade:** tension decays slowly instead of stepping — subtle
  and dangerous for an attitude-flying pilot (speed decays toward stall).
  Winchy's tension trace makes the fade visible at the winch.
- **Weak link logic:** the link (per the glider's POH, BGA tables) breaks
  between roughly 0.75–1.3× glider weight depending on type. The whole
  point of tension control is never to get there. Winchy's instrumented
  weak link must itself stay rated well above the break load of the
  sacrificial link next to it.

## Numbers at a glance

| Quantity | Typical value |
|---|---|
| Launch duration to ~500 m | 30–45 s |
| Release height | ≈ 45–50% of rope length (more in headwind) |
| Ground-roll acceleration | 0.5–1 g (weak-link ceiling ~1.5 g) |
| Lift-off speed / start rotation | 35–45 kt / ≥ 50 kt |
| Rotation rate / duration | ≤ 10°/s, 3–5 s |
| Climb angle (established) | 35–45° |
| Tension factor k = T/W | 0.7–1.5 (ATC setpoints); ~1.0 nominal |
| Tension for a 540 kg trainer @ k=1 | ≈ 5.3 kN |
| Weak link break load | 0.75–1.3 × W (per glider POH) |
| Dyneema rope | ~5 mm, ~22 kN break, ~1.2 kg/100 m |
| Rope speed change over launch | drops ~66% from peak |
| Back-release rope angle | 70–75° to glider longitudinal axis |
| Speed decay after break at 45° nose-up | ~12 kt/s |
| Peak winch power (1-seat trainer, sea level) | ~300 HP (more at altitude) |

## Consequences for Winchy's measurements

1. **Rope sag bias.** The rope hangs in a catenary (weight + aero drag), so
   the tension *direction* at the glider end differs from the straight line
   to the winch, and sag grows as tension falls. The Wojnar/Stołtny FEM
   study calls this the dominant systematic error when measuring tension
   vector and angles — Dyneema reduces sag ~80% vs steel but it is not
   zero. Our measured tension magnitude is correct *at our attachment
   point*; the IMU-derived angle is the local rope tangent, not the
   line-of-sight elevation. Treat them accordingly (and don't "correct" one
   with the other without a sag model).
2. **The accelerometer measures specific force,** gravity plus kinematic
   acceleration plus rope jerk and oscillation. During ground roll the
   horizontal component is the launch acceleration (good for mass
   estimation); during rotation/climb the gravity direction must come from
   fusing accel with the magnetometer/gyro (Kalman filter) because the unit
   is accelerating hard while rotating.
3. **Tension dynamics are fast.** Drivetrain-induced oscillations are the
   thing worth displaying/flagging; that requires force sampling well above
   10 Hz (ADS1232 supports 80 SPS — use it during the launch) even if
   telemetry to the ground runs at a few Hz with min/max/mean per frame.
4. **Phase detection is cheap and robust** from (tension, tension-rate,
   longitudinal accel, rope angle, altitude rate) — no GPS needed, which
   matters because the launch is over before a cold GPS gets a fix.
5. **The advice function in one sentence:** ramp smoothly to the
   glider-specific tension target, hold k·W constant through rotation and
   climb (which implies continuously reducing power as rope speed falls),
   then cut early at the top — and scream immediately if tension steps to
   zero or oscillates near the weak-link margin.

## Sources

- Bill Daniels, [Winch Launch Training Guidelines, Rev. Jan 2010](https://www.pas.rochester.edu/~cline/Winch%20launch%20training%20guide%20Rev%2012.pdf) — phases, rotation hazards, tension factor, failure recovery, ATC vs SC.
- Bill Daniels, [Building a Winch? Design considerations, 2008](https://www.pas.rochester.edu/~cline/FLSC/Winch%20construction%20paper.pdf) — power/torque math, rope-speed decay, tension oscillations, DAeC winch requirements, ATC control laws.
- T. Wojnar, B. Stołtny, [An analysis of rope tension forces while towing a sailplane, Safety & Defense 8(2), 2022](https://sd-magazine.eu/index.php/sd/article/download/182/132/) — force/moment balance on ground and airborne, FEM rope model, sag as systematic measurement error, Dyneema vs steel.
- Scottish Gliding Centre (Portmoak), [The Winch's Perspective](https://pilots.scottishglidingcentre.co.uk/doku.php/winch/the_winchs_perspective) — driver technique per phase, Skylaunch throttle guide, wind effects.
- [glidingschool.com — Winch Launch](https://glidingschool.com/winch-launch/) and [Briefing: Winch Launch](https://glidingschool.com/briefing-winch-launch/) — pilot procedure, signals, back-release.
- BGA Safe Winch Launching initiative ([brochure mirror](https://gliding.co.nz/wp-content/uploads/2016/06/BGA-safewinchbrochure-0210.pdf)) — accident drivers, rotation-rate guidance, weak links.
