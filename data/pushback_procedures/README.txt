BetterPushback - default pushback procedures
============================================

Files in this folder describe the published pushback procedure of airport
stands, one file per airport named after the airport ident used in apt.dat,
for example EDDF.txt. When the pushback planner opens at a stand that has an
entry, the procedure is offered in the planner (and can be loaded
automatically, see Preferences > "Auto-load stand default procedure").

Procedures are aircraft independent: each step ("push back 40 m", "turn so
the tail points east") is converted into a route for the aircraft you fly,
using the same planner as manual routes.

Your own defaults, saved with the planner's "Save as stand default" button,
are written to

    X-Plane/Output/preferences/BetterPushback_procedures/<ICAO>.txt

They use the same format, take precedence over the files in this folder and
survive plugin updates. Edit that folder rather than this one.

Please only enter procedures you transcribed from an official source (the
airport's AIP chapter AD 2.20 "Local aerodrome regulations" / AD 2.24 charts,
the airport operator's ground handling or apron manual, ...) or from an
online network's published pilot information, and record that source with
the 'source' keyword. Name network procedures after the network and date
(e.g. "VATSIM Scandinavia 2026-07-28: ...") and list them after the official
ones; the planner shows names starting with "VATSIM" under their own heading.
Do not use them for real-world operations.

YOU ARE RESPONSIBLE for the procedure you choose: check that it matches your
ATC instructions and the rules of your online network before pushing back.

Format (see EXAMPLE.txt; '#' starts a comment, keywords are case-insensitive)
-----------------------------------------------------------------------------

stand <pattern>
    Starts a procedure. <pattern> is matched against the apt.dat ramp start
    name (case-insensitive). '*' and '?' wildcards are allowed. The pattern
    is also tried against the last word of the name, so "A12" matches
    "Gate A12". Exact patterns are listed before wildcard patterns.
name <text>
    Label shown in the planner.
source <text>
    Where the procedure comes from (shown as a tooltip).
back <meters>
    Straight pushback.
forward <meters>
    Straight pull forward.
turn face <dir> [radius <m>] [cw|ccw] [forward]
turn tail <dir> [radius <m>] [cw|ccw] [forward]
    Turn while pushing back (or pulling, with 'forward') until the nose
    ('face') or the tail ('tail') points towards <dir>:
      - a compass point: N NNE NE ENE E ESE SE SSE S SSW SW WSW W WNW NW NNW
        or north, east, south, west, northeast, ...
      - a true heading in degrees: 090
      - a change of the nose heading: +90 or -45
      - left / right [angle]: "turn tail left" swings the tail to the
        pilot's left by 90 degrees, "turn face right 45" turns the nose 45
        degrees to the right.
    cw / ccw force the direction in which the nose heading changes
    (otherwise the shorter turn is used). Without 'radius' a 30 m radius is
    used and widened automatically if the aircraft cannot turn that tightly.
end
    Ends the procedure.
