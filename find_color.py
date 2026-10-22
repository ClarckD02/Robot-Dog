#!/usr/bin/env python3
"""
PiDog color finder: search for a color, stop and report it, optionally walk back.

Run on the PiDog (from ~/Robot-Dog):
    sudo python3 find_color.py red              # look for red, report, stay put
    sudo python3 find_color.py blue --return    # ...then walk back to the start
    sudo python3 find_color.py any --return     # look for ANY of the known colors

Colors: red, orange, yellow, green, blue, purple (what vilib's detector knows).
Live camera view: http://<PiDog IP>:9000/mjpg

How "go back" works: every move is logged, and returning replays the log in
reverse with each move undone (forward <-> backward, left <-> right). That is
dead reckoning, so it lands NEAR the start, not exactly on it. Walking
surfaces, battery and slipping all add error.


"""
import json
import sys
from time import sleep, strftime

from pidog import Pidog
from vilib import Vilib

# ---- Tunable settings -------------------------------------------------
KNOWN_COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'purple']
HEAD_SWEEP = [-60, -30, 0, 30, 60]   # head yaw angles to look at (degrees)
SETTLE_S = 0.4                       # wait after changing head angle / color
MIN_BLOB_WIDTH = 20                  # ignore color specks smaller than this (px)
MAX_BODY_TURNS = 8                   # body turns per spot before walking on
TURN_STEPS = 2                       # 'turn_left' steps per body turn
MAX_LEGS = 3                         # how many times to walk on and search again
WALK_STEPS = 3                       # 'forward' steps per leg
OBSTACLE_CM = 25                     # don't walk forward if closer than this
TURN_SPEED = 95                      # use whatever speed value kept your dog stable
WALK_SPEED = 95
RESULT_FILE = 'found_color.json'
# --------------------------------------------------------------------------

INVERSE = {'forward': 'backward', 'backward': 'forward',
           'turn_left': 'turn_right', 'turn_right': 'turn_left'}

my_dog = Pidog()
sleep(0.1)
moves = []   # every body move taken: (action, step_count)


def wait_done():
    my_dog.wait_legs_done()
    my_dog.wait_head_done()


class Tipped(Exception):
    pass


def check_tilt():
    """Stop if the body is tipped way over. Per SunFounder's IMU docs, ax is ~ -1g
    (-16384) when level, so a small |ax| means the dog is on its side or back."""
    try:
        ax, _, _ = my_dog.accData
    except Exception:
        return  # no IMU reading, can't judge
    if abs(ax) < 9000:
        raise Tipped()


def move(action, steps):
    speed = WALK_SPEED if action in ('forward', 'backward') else TURN_SPEED
    my_dog.do_action(action, step_count=steps, speed=speed)
    wait_done()
    moves.append((action, steps))
    check_tilt()


def set_color(color):
    if hasattr(Vilib, 'color_detect'):
        Vilib.color_detect(color)
    elif hasattr(Vilib, 'color_detect_switch'):
        Vilib.color_detect_switch(True)
    else:
        print('No color detection method found. Available Vilib methods:')
        print([m for m in dir(Vilib) if 'color' in m.lower() or 'detect' in m.lower()])
        raise SystemExit(1)


def color_visible():
    p = Vilib.detect_obj_parameter
    n = p.get('color_n')
    return (n is None or n > 0) and p.get('color_w', 0) >= MIN_BLOB_WIDTH


def seen_stable(checks=5, delay=0.1):
    """Require the color in most of several quick checks to ignore flicker."""
    hits = 0
    for _ in range(checks):
        hits += color_visible()
        sleep(delay)
    return hits >= 3


def look_for(colors):
    """Sweep the head. Returns (color, head_yaw) of the first hit, or None."""
    for yaw in HEAD_SWEEP:
        my_dog.head_move([[yaw, 0, 0]], immediately=True, speed=80)
        my_dog.wait_head_done()
        sleep(SETTLE_S)
        for color in colors:
            set_color(color)
            sleep(SETTLE_S)
            if seen_stable():
                return color, yaw
    my_dog.head_move([[0, 0, 0]], immediately=True, speed=80)
    my_dog.wait_head_done()
    return None


def search(colors):
    for leg in range(MAX_LEGS + 1):
        print(f'Searching from spot {leg + 1} of {MAX_LEGS + 1}...')
        for _ in range(MAX_BODY_TURNS + 1):
            hit = look_for(colors)
            if hit:
                return hit
            move('turn_left', TURN_STEPS)
        if leg < MAX_LEGS:
            d = my_dog.read_distance()
            if 0 < d < OBSTACLE_CM:
                print(f'Something is {d:.0f} cm ahead, turning instead of walking.')
                move('turn_left', TURN_STEPS * 2)
            else:
                move('forward', WALK_STEPS)
    return None


def report(color, yaw):
    print(f'\n*** FOUND {color.upper()} (head yaw {yaw} deg) ***')
    with open(RESULT_FILE, 'w') as f:
        json.dump({'color': color, 'head_yaw': yaw,
                   'time': strftime('%Y-%m-%d %H:%M:%S'),
                   'moves_from_start': len(moves)}, f)
    print(f'Saved to {RESULT_FILE}')
    my_dog.head_move([[yaw, 0, 0]], immediately=True, speed=80)
    my_dog.do_action('wag_tail', step_count=5, speed=95)
    my_dog.speak('single_bark_1')
    wait_done()


def go_back():
    print('Walking back to the start (approximate)...')
    for action, steps in reversed(list(moves)):
        my_dog.do_action(INVERSE[action], step_count=steps,
                         speed=WALK_SPEED if action in ('forward', 'backward') else TURN_SPEED)
        wait_done()
        check_tilt()
    moves.clear()
    my_dog.head_move([[0, 0, 0]], immediately=True, speed=80)
    my_dog.do_action('sit', speed=50)
    wait_done()
    print('Back at the start. Done.')


def main():
    args = [a.lower() for a in sys.argv[1:]]
    want_return = '--return' in args
    names = [a for a in args if not a.startswith('--')]
    target = names[0] if names else 'red'
    if target != 'any' and target not in KNOWN_COLORS:
        print(f'Unknown color "{target}". Use one of: {", ".join(KNOWN_COLORS)}, or any')
        raise SystemExit(1)
    colors = KNOWN_COLORS if target == 'any' else [target]

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=True, web=True)
    sleep(0.5)
    my_dog.do_action('stand', speed=50)
    wait_done()
    print(f'Looking for: {target}' + ('  (will return to start)' if want_return else ''))

    try:
        hit = search(colors)
        if hit is None:
            print('Did not find that color. Giving up.')
            if want_return:
                go_back()
            return
        report(*hit)
        if want_return:
            go_back()
    except Tipped:
        print('The dog tipped over! Stopping all movement.')
        try:
            my_dog.do_action('lie', speed=40)
        except Exception:
            pass
    finally:
        Vilib.camera_close()
        my_dog.close()


if __name__ == '__main__':
    main()