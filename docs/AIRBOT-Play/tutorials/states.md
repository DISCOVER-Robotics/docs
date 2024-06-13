# States of AIRBOT Play

## States of AIRBOT Play

After AIRBOT Play is powered on and controlled, it will enter different states according to the operation. The states are indicated by the color of the LED light on the base control board. 

The following table lists the states and their corresponding LED light colors.

| State | Color | Description |
| --- | --- | --- |
| Self-check | Breathing <br /><span style='color: yellow'><strong>Yellow</strong></span> | The robot is powered on and self-checking. |
| Power On | Constant <strong>White</strong> | The robot is powered on and prepared to be controlled |
| Online <br />(Idle) | Constant <br /><span style='color: green'><strong>Green</strong></span> | The robot is in online mode and can be controlled by external commands (e.g., keyboard, API calling) |
| Online <br />(Idle) | Flashing <br /><span style='color: green'><strong>Green</strong></span> | The robot is in online mode and is moving right now |
| Manual | Constant <br /><span style='color: cyan'><strong>Cyan</strong></span> | The robot is in manual mode and can be dragged freely with gravity compensation. |
| Manual <br />(Recording) | Constant <br /><span style='color: blue'><strong>Blue</strong></span> | The robot is in manual mode and can be dragged freely with gravity compensation. The trajectory is being recorded. |
| Offline <br />(Idle) | Constant <br /><span style='color: purple'><strong>Purple</strong></span> | The robot is in offline mode prepared to replay recorded trajectory. External commands (e.g., keyboard, API calling) will be ignored. |
| Offline <br />(Moving to starting point) | Flowing <br /><span style='color: purple'><strong>Purple</strong></span> | The robot is in offline mode, moving to the starting point of the recorded trajectory. External commands (e.g., keyboard, API calling) will be ignored. |
| Offline <br />(Replaying) | Breathing <br /><span style='color: purple'><strong>Purple</strong></span> | The robot is in offline mode replaying the recorded trajectory. |
| Error | Constant <br /><span style='color: Red'><strong>Red</strong></span> | The robot is in error state. |

## Switching between States

The states of AIRBOT Play can be switched by pressing buttons on the base panels and on the end:

| Current State | Action | Next State | Side Effect | Description |
| --- | --- | --- | --- | --- |
| Online <br />(Idle) | <span style="color: pink"><strong>LONG PRESS</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | Manual || Start gravity compensation and free drive the arm|
| Offline <br />(Idle) | <span style="color: pink"><strong>LONG PRESS</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | Manual || Start gravity compensation and free drive the arm |
| Manual | <span style="color: pink"><strong>LONG PRESS</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | Online <br />(Idle) || Stop gravity compensation and ready for external control |
| Online <br />(Idle) | <span style="color: tan"><strong>DOUBLE CLICK</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | Offline <br />(Idle) || Stop responding to external control and prepare for trajectory replay |
| Offline <br />(Idle) | <span style="color: tan"><strong>DOUBLE CLICK</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | Online <br />(Idle) || Return to online mode accepting external control |
| Manual |<span style="color: tan"><strong>DOUBLE CLICK</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | Offline <br />(Idle)  || Stop gravity compensation and prepare for trajectory replay |
| Offline <br />(Idle) | <span style="color: #66CDAA"><strong>SINGLE CLICK</strong></span> on the <span style="color: teal"><strong>BASE BUTTON</strong></span> | Offline <br />(Moving to starting point) || Prepare to replay recorded trajectory by moving to the starting point |
| Manual | <span style="color: #66CDAA"><strong>SINGLE CLICK</strong></span> on the <span style="color: teal"><strong>BASE BUTTON</strong></span> | Manual <br />(Recording) || Start recording trajectory|
| Manual <br />(Recording) | <span style="color: #66CDAA"><strong>SINGLE CLICK</strong></span> on the <span style="color: teal"><strong>BASE BUTTON</strong></span> | Manual || Stop recording trajectory|
| Manual or Manual <br />(Recording) | <span style="color: #66CDAA"><strong>SINGLE CLICK</strong></span> on the <span style="color: peru"><strong>END BUTTON</strong></span> | <span style="color: gray">*UNCHANGED*</span> | Gripper opened / closed | Open / close the gripper while dragging the end effector|

The states can also be switched by external commands:

* `manual_mode()`: Switch to manual mode
* `online_mode()`: Switch to online mode
* `offline_mode()`: Switch to offline mode
* `record_start()`: Start recording trajectory
* `record_stop()`: Stop recording trajectory
* `replay_start()`: Start replaying recorded trajectory