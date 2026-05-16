#!/usr/bin/env python3
"""
Replay the autonomous_mission STATUSTEXT sequence in QGC — no drone needed.

Starts a MAVLink UDP server on port 14550.  QGC connects to it, receives a
heartbeat stream, and sees the full mission state sequence appear in the
Messages panel exactly as it will during a real flight via the SiK radio.

Usage:
    python3 tools/test_statustext.py

Then in QGC:
    Application Settings → Comm Links → Add
    Type: UDP   Port: 14550   Target Host: <RPi IP shown below>
    Connect → watch the Messages panel (bell icon, top-right)
"""
import time
import sys
from pymavlink import mavutil

PORT     = 14550
SYS_ID   = 1    # same MAVLink system as the autopilot
COMP_ID  = 191  # MAV_COMP_ID_ONBOARD_COMPUTER — not the autopilot

# Mission events: (text, seconds to hold before next event)
# Mirrors the _statustext() calls in autonomous_mission.py exactly.
EVENTS = [
    ('Mission: ARMING',          4.0),
    ('Mission: TAKEOFF',         6.0),
    ('Mission: FLY_PICKUP',      9.0),
    ('Mission: VISUAL',          8.0),
    ('GRABBED [blue]',           1.0),
    ('Mission: CLIMB',           6.0),
    ('Mission: FLY_DROP',        9.0),
    ('Mission: DROP',            4.0),
    ('RELEASED at drop zone',    2.0),
    ('Mission: RTL',            15.0),
    ('Mission: DONE',            0.0),
]


def _statustext(conn, text: str):
    text_bytes = (text + '\x00' * 50)[:50].encode('ascii')
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_bytes)
    print(f'  [STATUSTEXT] {text}')


def _heartbeat(conn):
    # Identify as onboard computer, not autopilot — QGC shows STATUSTEXT
    # from any system but only announces flight state for MAV_AUTOPILOT != INVALID.
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def main():
    import socket
    hostname = socket.gethostname()
    try:
        ip = socket.gethostbyname(hostname)
    except Exception:
        ip = '(run `hostname -I` to find it)'

    print('═' * 60)
    print(' MAVLink STATUSTEXT test server')
    print('═' * 60)
    print(f' Listening on UDP port {PORT}')
    print()
    print(' In QGC:')
    print('   Application Settings → Comm Links → Add')
    print(f'   Type: UDP   Port: {PORT}   Target Host: {ip}')
    print('   Click Connect, then watch the Messages panel (🔔 top-right)')
    print('═' * 60)
    print()

    conn = mavutil.mavlink_connection(
        f'udpin:0.0.0.0:{PORT}',
        source_system=SYS_ID,
        source_component=COMP_ID,
    )

    print('Waiting for QGC to connect...', flush=True)
    conn.wait_heartbeat(timeout=120)
    print(f'QGC connected — starting mission replay in 3 s...\n')
    time.sleep(3.0)

    _heartbeat(conn)
    _statustext(conn, 'PIES: mission replay starting')
    time.sleep(1.0)

    for text, hold_s in EVENTS:
        _heartbeat(conn)
        _statustext(conn, text)

        # Keep heartbeating during the hold so QGC stays connected
        elapsed = 0.0
        while elapsed < hold_s:
            time.sleep(1.0)
            elapsed += 1.0
            _heartbeat(conn)

    print('\nReplay complete — all states sent.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nAborted.')
