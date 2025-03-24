# Controller bindings

## Driver 

### Joysticks

- Left joystick:
    Moves the bot

- Right joystick:
    Rotates the bot

### Buttons

- L2: Rotates climber wheels inwards
    - Rotates left wheel clockwise, right wheel counterclockwise.
      Hold to rotate

- R2: Rotates climber wheels outwards?
    - Rotates left wheel counterclockwise, right wheel clockwise.
      Hold to rotate

- R1: Puts driving into "surgery mode"
    - In surgery mode, the bot moves and turns slower, allowing
      for finer adjustment and positioning via joysticks

- Circle:
    - Resets the bot's relative rotation to that of the operator's perspective,
      making forwards on the joystick the bot forward and so on

- Cross:
    - Resets the bot's relative rotation to that of the operator's perspective minus
      180 degrees, making forward on the joystick drive the bot backwards and so on

- Square:
    - Resets the bot's relative rotation to that of the operator's perspective plus
      90 degrees, making forward on the joystick drive the bot left and so on

- Triangle:
    - Resets the bot's relative rotation to that of the operator's perspective minus
      90 degrees, making forward on the joystick drive the bot right and so on

## Mate

### Joysticks

- Right joystick:
    - Wrist control. Moving the joystick up and down moves the wrist up and down.

### Buttons

- R1 and L1:
    - Winch control. R1 releases winch, L1 pulls it in.

- R2 and L2:
    - Intake motor control. R2 spins wheels in, L2 spins them out.

- DPad up and down:
    - Controls elevator height. DPad up moves it up one level, DPad down moves it down one level.

- Circle:
    - Resets subsystem positions to default, moving the elevator, arm, and wrist to their safe starting positions.

- Triangle:
    - Moves the arm out to placing position.

- Square:
    - Moves the arm out to intaking position.
    - Retracts funnel?

- Share:
    - Cancels current running command.

- Touchpad:
    - Climbing Position: Pulls folder back, moves arm and wrist vertical

--- 