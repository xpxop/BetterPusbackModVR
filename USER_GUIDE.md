# BetterPushback User Guide

BetterPushback provides a pilot-guided pushback and towing workflow for
X-Plane 11 and 12. The Ground Operations window follows the tug from dispatch
through connection, pushback, disconnect, and final clearance while preserving
BetterPushback's established route planning and steering behavior.

This guide describes the owner-review release candidate. Availability on a
specific operating system or X-Plane version depends on the final release
package and its published compatibility notes.

## 1. Install and open

1. Close X-Plane before replacing an existing plugin installation.
2. Back up the current `BetterPushback` plugin folder and any saved route data.
3. Copy the complete release `BetterPushback` folder into
   `X-Plane/Resources/plugins`.
4. Start X-Plane and load the aircraft at a stand. Ground Operations opens
   automatically when BetterPushback connects to the simulator.

Do not copy only the `.xpl` binary. Tug objects, sounds, configuration files,
the wing walker, and supporting resources are part of the plugin.

## 2. Read the progress rail

Ground Operations uses five stages:

| Stage | Meaning |
| --- | --- |
| Tug | Tug selection, dispatch, approach, and service-area checks |
| Connect | Final approach, nose-gear capture, and lift |
| Comms | Route planning and brake-release coordination |
| Push | Pushback or tow movement, Pause, Resume, and stopping |
| Clear | Lowering, disconnect, tug clearance, and the final hand signal |

The node colors are operational cues:

| Appearance | Meaning | Pilot response |
| --- | --- | --- |
| Bright green with a strong ring | The current stage is progressing normally | Monitor the operation |
| Amber/orange | Progress is blocked on a required pilot action | Expand the panel and follow the PILOT ACTION card |
| Green | The stage is complete | No action required |
| Cyan outline | The stage is still in the future | No action required |

Red is not used for normal operation. It is reserved for a future explicit
fault or safety-stop condition. A destructive alternative such as **End
operation** does not make the stage red.

Color is never the only status indicator. The stage name, status, CURRENT TASK
or PILOT ACTION heading, task wording, and button label describe what is
happening.

## 3. Window modes

- **Compact rail:** Shows only the five stages. Click the rail to expand it.
  Dragging moves it without expanding. Compact mode intentionally has no
  tooltips.
- **Expanded panel:** Shows airport and weather context, current status, task,
  speed, remaining distance, captions, and available controls.
- **Collapsed:** Click the minus control, hold the pointer over it for about one
  second, or click the expanded stage rail.
- **Pop-out:** Use the pop-out control to move Ground Operations into a native
  operating-system window, including another display.
- **Hidden:** Use the X control or the show/hide command. Hiding the window does
  not cancel the operation. The window remains hidden for the current session
  until the pilot shows it again, but opens automatically at the next simulator
  start.

At simulator start, Ground Operations is visible while the aircraft is stopped
on the ground. Outside an active operation, it automatically hides once the
aircraft begins taxiing and restores when the aircraft stops. This applies to GA
aircraft and airliners. Using the show command while moving is a manual override
that keeps the window visible until the pilot hides it again.

Positions are remembered by window mode, monitor, and side. If a saved display
is removed or resized, the window is recovered into a visible area.
The last visible compact or expanded presentation is also remembered. A first
install, or an older saved hidden state without that history, opens expanded.

The Preferences window provides **Standard**, **Large**, and **Extra large**
Ground Operations sizes. The selected size scales the complete compact rail or
expanded panel, including its text, controls, spacing, and click targets, so
larger text is not forced back into the original fixed-size layout.

The global **Pushback interface** preference selects either **Ground
Operations** or the original **Legacy magic squares**. The selection changes
immediately while BetterPushback is idle and is retained after **Save
preferences**. Only one operational interface runs at a time. Changing this
preference does not change the planner, tug physics, route cache, or pushback
controller. Ground Operations size and auto-expand settings are preserved while
legacy mode is selected and return unchanged when Ground Operations is selected
again.

In Legacy magic squares mode, use **Magic squares position** in Preferences to
move the complete stack vertically. This is the original legacy placement
control; the squares themselves are buttons and are not dragged directly.

The global **Auto-expand for pilot actions** preference is off by default. When
enabled, a compact visible rail expands when a new pilot action is required and
collapses one second after that action is completed. The completed action
collapses the expanded panel whether it was opened automatically or manually.
It never reopens a hidden window. A second required action during the delay
keeps the panel expanded. With the preference disabled, expand and collapse are
entirely manual. Toggling the option takes effect immediately; select **Save
preferences** to retain it across simulator starts.

The global **Auto disconnect when done** preference controls the final
post-push interaction for every aircraft and is remembered across simulator
starts. It is off by default, preserving the manual disconnect and clear-signal
acknowledgements described below.

## 4. Normal pushback

### Call the tug

The operation begins at an amber/orange **Tug** node and a **Call tug** PILOT
ACTION. Nothing is dispatched automatically. Press **Call tug** when the
aircraft and ramp are ready.

After the call is accepted, the Tug node changes to bright green. Tug selection,
dispatch, approach, and connection preparation proceed automatically. The panel
explains any door, GPU, ASU, or parking-brake condition that prevents progress.

### Plan the push

After the tug captures the nose gear, it stops before lifting. The **Comms** node
turns amber/orange and the panel presents **Plan push**. Open the overhead
planner, place the route manually, and accept it.

The blue line is the planned nosewheel trajectory. The translucent magenta band
is a wingspan-wide visual-clearance aid. It is not automatic collision
detection; the pilot remains responsible for verifying the route and clearance.

### Plan in the map window (VR)

The overhead planner moves the X-Plane camera and needs a mouse and keyboard,
so it cannot be used with a VR headset. In VR the planner therefore opens as a
**map window** that X-Plane places in the cockpit like other plugin windows.
Choose the behavior in **Preferences > Pushback planner view**:

| Setting | Planner |
| --- | --- |
| Map window in VR (default) | Map window in VR, overhead camera on a monitor |
| Always map window | Map window everywhere |
| Always top-down camera | The classic overhead planner |

The map is drawn from the active airport's `apt.dat` (custom scenery first, in
`scenery_packs.ini` order, then the global airports): pavement, painted taxi
lines, runways and stands. Airports without painted lines show their taxi
routing network instead. Your aircraft is drawn in white, the end of the
committed route in green and the target of the next segment in amber (red when
the target cannot be reached).

1. Click the map where the next segment should end. With **Snap to taxi
   lines** on, the target heading aligns with the nearest taxi line.
2. Adjust the target heading with the ±5/±15 buttons, **Tail left 90** /
   **Tail right 90**, or by clicking the compass dial.
3. Press **Add segment**. Repeat for further segments, or use **Undo** /
   **Clear**.
4. Press **Accept route**. **Connect tug first** and **Cancel** work as in the
   overhead planner.

Drag the map to pan and use the mouse wheel or **Zoom +/-** to zoom;
**Center** returns to the aircraft and **Heading up** rotates the map with the
aircraft. With **Click adds segment** enabled a click commits the segment
immediately, like the overhead planner. Saved-route choices appear as buttons
on the map.

### Release the parking brake

When the tug is connected and the route is ready, the panel displays the next
required brake action. The Comms node remains amber/orange until the required
condition is satisfied. A **Change plan** control may be available while the
aircraft is connected and held by the parking brake; it is an optional control,
not the required action itself.

### Monitor, pause, or end

During a normal push the **Push** node is bright green. **Pause push** requests a
controlled deceleration and retains the accepted route and steering state.
After the tug reaches a stationary pause, **Resume push** becomes a required
pilot action and the Push node is amber/orange.

**End operation** is different from Pause. It discards the remaining route and
continues through a safe stop and disconnect sequence at the current position.
The confirmation step prevents accidental activation.

### Disconnect and clear

At the route end, set the parking brake when requested. The tug lowers and
releases the nose gear. At **Ready to disconnect**, choose **Disconnect tug** to
continue or **Reconnect** if the connection must be restored.

The global **Display marshaller** preference controls whether the wing walker
is used during normal operations and is enabled by default. When displayed, he
appears at the established 30-yard nose clearance, faces the aircraft, and
remains fixed at that original ground position even if the aircraft begins to
taxi before he disappears. He presents STOP, STANDBY, and CLEAR signals. When
the clear signal is visible, **Acknowledge** it in the panel.
Acknowledgement does not shorten the established minimum signal-display time.
After the tug departs, every stage is green and the operation is complete.

With **Auto disconnect when done** enabled, the workflow is unchanged through
the request to set the parking brake. After the brake is set, BetterPushback
automatically approves **Disconnect tug** and acknowledges the final pin/clear
signal. No further pilot action is required, but the tug still lowers, releases,
moves to the selected side, presents its signals, and observes the same 15-second
minimum clear-signal display before departure.

## 5. Saved routes

At a published airport start, BetterPushback can identify a gate or stand only
when the live nosewheel position and heading uniquely match it within the
plugin's strict tolerance. A matching airport, stand, and compatible aircraft
profile can have two independent route slots.

- Select an existing slot to reuse a route.
- Select an explicit slot when replacing a saved route.
- Accepting an unchanged recalled route does not rewrite it.
- Arbitrary-position, off-anchor, saved-situation, and Emergency Tow routes are
  session-only and cannot list, load, replace, or save persistent slots.

If no unique published start is recognized, plan the route normally for the
current operation. The panel and log explain why persistent saving is disabled.

BetterPushback checks the active scenery order and the existence, size, and
modification time of each active `apt.dat` when the plugin starts. If custom or
global airport scenery has changed, its derived airport database is rebuilt
automatically. Saved push-route slots are stored separately and are not removed
by this rebuild. The first startup after a scenery change can therefore take
longer than usual.

### Stand default procedures

Besides the two per-aircraft route slots, each stand can have **default
pushback procedures** that work for every aircraft. A procedure is a short list
of steps such as "push back 20 m, turn until the tail points east, push back
10 m"; BetterPushback turns it into a route for the current aircraft with the
normal planner, widening the turns automatically if the aircraft cannot turn
as tightly as written.

- In the map planner, the **Stand procedures** list shows the procedures for
  the stand you are parked at. Click one to load it, then review, edit or
  accept it.
- **Save as stand default** stores the current route as your default for the
  stand. Your defaults are written to
  `Output/preferences/BetterPushback_procedures/<ICAO>.txt` and are listed
  first (marked `*`).
- Procedures transcribed from official sources (AIP AD 2.20, airport apron or
  ground handling manuals) can be placed in
  `plugins/BetterPushback/data/pushback_procedures/<ICAO>.txt`. The format is
  described in `README.txt` in that folder, with examples in `EXAMPLE.txt`.
- **Preferences > Auto-load stand default procedure** loads the first matching
  procedure automatically when the map planner opens and no saved-route choice
  is pending.

The stand is identified from the recognized published start; otherwise the
nearest `apt.dat` stand within 40 m of the aircraft is used.

**Official and VATSIM procedures.** For the Norwegian airports the list shows
the official procedures (from the AIP) first, then the procedures published by
the VATSIM Scandinavia wiki under the heading *VATSIM (online network, not
official)*, named `VATSIM Scandinavia <date>: ...`. Choose the one that fits
how you fly. Hover a procedure to see its source.

> **You are responsible for the procedure you choose.** The procedures are for
> flight simulation only. Always check that the chosen procedure matches your
> ATC instructions and the rules of your online network (for VATSIM: the
> vACC's pilot briefing or wiki). The map planner shows this reminder above
> the list.

## 6. Emergency Tow

After a completed normal operation, **Call tow back** starts a guarded one-time
Emergency Tow. The tug returns, reconnects, and opens the manual planner at the
aircraft's live position.

Emergency Tow uses no saved routes and does not display the wing walker. Pause,
Resume, and End operation remain available. After final disconnect and tug
departure, BetterPushback returns to its normal cold-start state.

## 7. Commands and recovery

The most useful assignable commands are:

| Command | Purpose |
| --- | --- |
| `BetterPushback/ground_ops_show_hide` | Show or hide Ground Operations |
| `BetterPushback/ground_ops_expand_collapse` | Switch between compact and expanded views |
| `BetterPushback/start` | Start pushback or reopen Change plan during the connected hold |
| `BetterPushback/pause_resume` | Pause or resume without discarding the route |
| `BetterPushback/stop` | End pushback and disconnect |
| `BetterPushback/start_planner` | Open the manual planner |
| `BetterPushback/stop_planner` | Close the planner |
| `BetterPushback/acknowledge_clear` | Acknowledge the final clear signal |

Classic menu commands remain available as a recovery path. If the Ground
Operations window cannot initialize, the operational commands still remain
available.

## 8. Offline behavior, privacy, and diagnostics

The normal pushback workflow uses simulator-local aircraft, airport, and weather
context and remains usable offline. Ground Operations shows the source and
freshness of the displayed context.

Runtime telemetry recording is disabled in normal release builds. Diagnostic
builds can enable bounded per-operation CSV telemetry for troubleshooting. Such
files remain local unless the user deliberately shares them.

When reporting a problem, include the X-Plane version, operating system,
aircraft, airport and stand, display arrangement, the exact workflow step, and
the relevant `Log.txt`. Keep a backup of the previous working plugin until the
new release has passed a complete local pushback.

## 9. Current release scope

The current owner-review candidate includes:

- compact and expanded Ground Operations presentations, including pop-out and
  remembered multi-monitor placement;
- explicit Call tug, Plan push, and brake gates, with manual disconnect and
  clear-signal gates unless the global automatic option is enabled;
- manual route planning with the legacy route-following behavior;
- two guarded saved-route slots per matching gate/stand and aircraft profile;
- Pause/Resume and safe End operation behavior;
- normal-operation wing-walker signals and one-time Emergency Tow;
- simulator-local flight and weather context;
- default-off runtime telemetry and classic-command recovery paths.

Release acceptance still depends on the owner's final review, supported-platform
builds, packaging checks, and live simulator validation.

## 10. Future roadmap

The following items are candidates for separate owner approval and validation,
not promises for the current release:

- user-selectable display language with an Automatic/X-Plane-language option,
  explicit overrides, English fallback, and locale-specific short rail labels;
- optional SDK 4.4 native ImGui rendering with an earlier-runtime fallback;
- a separately scoped projected-ImGui planner overlay prototype;
- broader Windows, macOS/Metal, Linux, X-Plane 11/12, multi-display, scaling,
  VR, and long-session validation;
- additional accessibility, recovery, and tug-driver presentation refinements
  supported by measurable acceptance criteria.

Operational behavior changes remain separate from renderer modernization so
each change can be reviewed, measured, and rolled back independently.

## 11. Safety notice and credits

BetterPushback is an entertainment add-on. It is not intended for flight
training, real-world aircraft operation, or safety-critical use.

Original Better Pushback by Saso Kiselkov. This maintained fork and its release
materials preserve the repository's existing copyright, license, attribution,
and third-party asset notices.
