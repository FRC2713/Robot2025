# Todo list

- PID control of the intake arm
- Intake roler code (note: uses Krackens) (WIP)
- colllect coral off of ground & detect when collected
- Shoulder works same
- Pivot is removed
- Algae & coral rollers are the same motor & system now (note: also uses Krackens)
- Re-tune set ponts & scoring routines
- Handoff logic (intake to claw)
- Maybe add climber (if possible, and design and manufacturing do it)

- Intake
  - Intake has X44 for running rollers and moving intake vertically
  - Has ranged sensor to detect when coral is present
  - Bring down to ground & turn rollers on, wait until sensor sees coral (intakeCoral)
  - Second command called handoffPrep, holds coral, brings intake up to handoff position
  - Handoff command- runs rollers in reverse to spit coral into claw
  - Has X60 for motor

- Claw
  - Claw also needs a handoffPrep & handoff command
  - Claw also has sensor to detect stuff
  - Uses X60

- Handoff command
  - Do first (at the same time):
    - Intake needs to come up to handoff position
    - Claw needs to move to handoff position
    - Elevator needs to go to handoff position
  - Do second (at the same time):
    - Intake needs to spit coral
    - Claw needs to intake
