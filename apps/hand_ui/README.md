# Linker Hand NiceGUI integration

This directory vendors the manual hand UI from `lychee_barmate`.  The ROS
backend was adapted for this workcell: commands go to
`/linker_hand_bridge/<side>/manual_command`, then pass through the existing
model validation, freshness watchdog and slew limiter before the vendor driver.
It never publishes directly to the O30i/G20 command bus.

Run `ops/run/run_hand_ui.sh` and open <http://127.0.0.1:8080>.  The launcher
refuses to start while the normal operator backend owns TCP port 5590, avoiding
two simultaneous hand command sources.
