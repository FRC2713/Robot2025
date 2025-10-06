# Todo list

- PID control of the intake arm: api done, not tuned
- Intake roler code (note: uses Krackens): kracken code done, untested
- colllect coral off of ground & detect when collected: not started
  - Intake roller limit switch (resistance > amount = coral picked up): not started (subgoal)
- Shoulder works same: non-issue (just don't change anything)
- Algae & coral rollers are the same motor & system now (note: also uses Krackens): new people reprogramming arm (?)
- Re-tune set ponts & scoring routines: not started
- Handoff logic (intake to claw): not started
- Remove climber: not started

- Change variables with IP (Intake Pivot) acronym to something that makes more sense: not started (maybe won't be done)

- Intake
  - Intake has X60 for running rollers and X44 moving intake vertically (?): non-issue (uses same API)
  - Has ranged sensor to detect when coral is present: not started
  - Bring down to ground & turn rollers on, wait until sensor sees coral (intakeCoral): not started
  - Second command called handoffPrep, holds coral, brings intake up to handoff position: not started
  - Handoff command- runs rollers in reverse to spit coral into claw: not started

- Claw (on arm)
  - Claw also needs a handoffPrep & handoff command
  - Claw also has sensor to detect stuff
  - Uses X60
  - new people working on arm (?)

- Handoff command
  - Do first (at the same time):
    - Intake needs to come up to handoff position
    - Claw needs to move to handoff position
    - Elevator needs to go to handoff position
  - Do second (at the same time):
    - Intake needs to spit coral
    - Claw needs to intake
