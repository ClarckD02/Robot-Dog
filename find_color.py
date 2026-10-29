#!/usr/bin/env python3
"""
PiDog color finder: search for a color, stop and report it, optionally walk back.

Run on the PiDog (from ~/Robot-Dog):
    sudo python3 find_color.py red              # look for red, report, stay put
    sudo python3 find_color.py blue --return    # ...then walk back to the start
    sudo python3 find_color.py any --return     # look for ANY of the known colors

Colors: red, orange, yellow, green, blue, purple (what vilib's detector knows).
Live camera view: http://<PiDog IP>:9000/mjpg

How it walks: like examples/3_patrol.py. Steps are queued ahead of time and
the dog is never made to wait between them, so the gait flows from one step
into the next instead of stopping and restarting. The camera and the distance
sensor are checked while the legs keep moving. The dog only stops when it
sees the color (then it looks around with its head to make sure) or when the
search is over.

How "go back" works: every finished step is logged, and returning replays the
log in reverse with each move undone (forward <-> backward, left <-> right).
That is dead reckoning, so it lands NEAR the start, not exactly on it. Walking
surfaces, battery and slipping all add error.
"""
import json
import sys
from time import sleep, strftime, time

from pidog import Pidog
from vilib import Vilib

# ---- Tunable settings -------------------------------------------------
KNOWN_COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'purple']
HEAD_SWEEP = [0, -30, 30, -60, 60]   # head yaw angles to check after a sighting
SETTLE_S = 0.4                       # wait after changing head angle / color
MIN_BLOB_WIDTH = 20                  # ignore color specks smaller than this (px)
SPIN_STEPS = 16                      # 'turn_left' steps to look all the way around
MAX_LEGS = 3                         # how many times to walk on and search again
WALK_STEPS = 3                       # 'forward' steps per leg
OBSTACLE_CM = 25                     # turn instead of walking if closer than this
SPEED = 98                           # same speed as 3_patrol.py, for every move
AHEAD = 2                            # steps kept queued so the legs never stop
RESULT_FILE = 'found_color.json'
# --------------------------------------------------------------------------

INVERSE = {'forward': 'backward', 'backward': 'forward',
           'turn_left': 'turn_right', 'turn_right': 'turn_left'}

my_dog = Pidog()
sleep(0.1)
moves = []   # every finished body move: [action, step_count]


def wait_done():
    my_dog.wait_legs_done()
    my_dog.wait_head_done()


class Tipped(Exception):
    pass


def check_tilt():
    """Stop if the body is tipped way over. Per SunFounder's IMU docs, ax is ~ -1g
    (-16384) when level, so a small |ax| means the dog is on its side or back."""
    try:
        ax, ay, az = my_dog.accData
    except Exception:
        return  # no IMU reading, can't judge
    if ax == ay == az == 0:
        return  # IMU has not sent a reading yet
    if abs(ax) < 9000:
        raise Tipped()


def log_move(action):
    if moves and moves[-1][0] == action:
        moves[-1][1] += 1
    else:
        moves.append([action, 1])


class Walker:
    """Feeds steps into PiDog's leg queue without waiting in between, like
    3_patrol.py does, and keeps track of which steps have actually finished."""

    def __init__(self):
        self.frames = {}    # action -> frames in one step
        self.pending = []   # queued (action, tag) steps not yet finished, oldest first

    def _frames(self, action):
        if action not in self.frames:
            self.frames[action] = len(my_dog.actions_dict[action][0])
        return self.frames[action]

    def _sync(self):
        """Move finished steps from pending into the move log."""
        left = len(my_dog.legs_action_buffer)
        done = sum(self._frames(a) for a, _ in self.pending) - left
        while self.pending and self._frames(self.pending[0][0]) <= done:
            done -= self._frames(self.pending[0][0])
            log_move(self.pending.pop(0)[0])

    def update(self):
        with my_dog.legs_thread_lock:
            self._sync()

    def queued(self):
        return len(self.pending)

    def add(self, action, tag=None):
        """Queue one more step behind the ones already queued. The tag lets
        the caller tell its own steps apart from others in the queue."""
        self._frames(action)  # look this up before touching the queue
        my_dog.do_action(action, step_count=1, speed=SPEED)
        self.pending.append((action, tag))

    def cut(self):
        """Let the step in progress finish, drop the queued steps after it.
        Returns the dropped (action, tag) steps."""
        with my_dog.legs_thread_lock:
            self._sync()
            if len(self.pending) < 2:
                return []
            dropped = self.pending[1:]
            keep = len(my_dog.legs_action_buffer) - sum(self._frames(a) for a, _ in dropped)
            del my_dog.legs_action_buffer[max(keep, 0):]
            self.pending = self.pending[:1]
            return dropped

    def stop(self):
        """Finish the step in progress, then stand still. Returns dropped steps."""
        dropped = self.cut()
        while not my_dog.is_legs_done():
            check_tilt()
            sleep(0.01)
        self.update()
        return dropped


walker = Walker()


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


class Spotter:
    """Quick color check that runs while walking. With several colors it
    switches to the next color each time the current one has had SETTLE_S
    seconds to show up."""

    def __init__(self, colors):
        self.colors = colors
        self.i = 0
        self._switch(0)

    def _switch(self, i):
        self.i = i
        set_color(self.colors[i])
        self.since = time()

    def check(self):
        if time() - self.since < SETTLE_S:
            return None
        if color_visible():
            return self.colors[self.i]
        if len(self.colors) > 1:
            self._switch((self.i + 1) % len(self.colors))
        return None

    def resume(self):
        """Go back to cycling colors after a head-sweep check."""
        self._switch(self.i)


def look_for(color):
    """Standing still, sweep the head. Returns head yaw where the color is, or None."""
    set_color(color)
    for yaw in HEAD_SWEEP:
        my_dog.head_move([[yaw, 0, 0]], immediately=True, speed=95)
        my_dog.wait_head_done()
        sleep(SETTLE_S)
        if seen_stable():
            return yaw
    my_dog.head_move([[0, 0, 0]], immediately=True, speed=95)
    my_dog.wait_head_done()
    return None


def go(action, steps, spotter, avoid=False):
    """Queue `steps` steps of `action` without stopping between them. While the
    legs move, watch for the color and (if avoid) for obstacles. Returns
    (color, head_yaw) on a confirmed sighting, else None once all steps are
    queued, so the next call can add its steps behind them with no pause."""
    tag = object()   # marks the steps this call queues
    queued = 0
    turns = 0
    while queued < steps:
        walker.update()
        check_tilt()

        if avoid:
            d = my_dog.read_distance()
            if 0 < d < OBSTACLE_CM:
                if (action, tag) in walker.pending[1:]:
                    print(f'Something is {d:.0f} cm ahead, turning instead of walking.')
                    queued -= walker.cut().count((action, tag))
                if walker.queued() < AHEAD:
                    if turns >= SPIN_STEPS:
                        print('Boxed in, giving up on this leg.')
                        return None
                    walker.add('turn_left')
                    turns += 1
            elif walker.queued() < AHEAD:
                walker.add(action, tag)
                queued += 1
        elif walker.queued() < AHEAD:
            walker.add(action, tag)
            queued += 1

        color = spotter.check()
        if color:
            print(f'Saw something {color}, stopping to check...')
            queued -= walker.stop().count((action, tag))
            yaw = look_for(color)
            if yaw is not None:
                return color, yaw
            print('False alarm, moving on.')
            spotter.resume()

        sleep(0.01)
    return None


def search(colors):
    spotter = Spotter(colors)
    my_dog.rgb_strip.set_mode('breath', 'white', bps=0.5)
    for leg in range(MAX_LEGS + 1):
        print(f'Searching from spot {leg + 1} of {MAX_LEGS + 1}...')
        hit = go('turn_left', SPIN_STEPS, spotter)
        if hit:
            return hit
        if leg < MAX_LEGS:
            hit = go('forward', WALK_STEPS, spotter, avoid=True)
            if hit:
                return hit
    # all steps are queued; keep watching until the last one finishes
    while not my_dog.is_legs_done():
        check_tilt()
        color = spotter.check()
        if color:
            walker.stop()
            yaw = look_for(color)
            if yaw is not None:
                return color, yaw
            spotter.resume()
        sleep(0.01)
    walker.update()
    return None


def report(color, yaw):
    print(f'\n*** FOUND {color.upper()} (head yaw {yaw} deg) ***')
    with open(RESULT_FILE, 'w') as f:
        json.dump({'color': color, 'head_yaw': yaw,
                   'time': strftime('%Y-%m-%d %H:%M:%S'),
                   'moves_from_start': sum(n for _, n in moves)}, f)
    print(f'Saved to {RESULT_FILE}')
    my_dog.rgb_strip.set_mode('boom', 'green', bps=1)
    my_dog.head_move([[yaw, 0, 0]], immediately=True, speed=80)
    my_dog.do_action('wag_tail', step_count=5, speed=95)
    my_dog.speak('single_bark_1')
    my_dog.wait_all_done()


def go_back():
    print('Walking back to the start (approximate)...')
    my_dog.head_move([[0, 0, 0]], immediately=True, speed=80)
    my_dog.rgb_strip.set_mode('breath', 'white', bps=0.5)
    # queue the whole trip at once so the legs never stop between moves
    for action, steps in reversed(moves):
        my_dog.do_action(INVERSE[action], step_count=steps, speed=SPEED)
    while not my_dog.is_legs_done():
        check_tilt()
        sleep(0.01)
    moves.clear()
    my_dog.do_action('sit', speed=95)
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
    Vilib.display(local=False, web=True)
    sleep(0.5)
    # same start as 3_patrol.py
    my_dog.do_action('stand', speed=80)
    my_dog.wait_all_done()
    sleep(0.5)
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
            my_dog.body_stop()
            my_dog.do_action('lie', speed=95)
        except Exception:
            pass
    finally:
        Vilib.camera_close()
        my_dog.close()


if __name__ == '__main__':
    main()
