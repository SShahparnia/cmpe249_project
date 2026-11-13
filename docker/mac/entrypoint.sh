#!/usr/bin/env bash
# Start the browser desktop (Xvfb :1 -> x11vnc -> noVNC on :6080), then run CMD.
set -e

Xvfb :1 -screen 0 1920x1080x24 +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
for _ in $(seq 50); do [ -e /tmp/.X11-unix/X1 ] && break; sleep 0.1; done

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :1 -nopw -forever -shared -localhost -rfbport 5900 -quiet >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &

exec "$@"
