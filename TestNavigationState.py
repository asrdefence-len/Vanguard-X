from NavigationState import SimulatedNavigationSource
import time

nav = SimulatedNavigationSource(initial_heading_deg=10.0)
a = nav.get_attitude()
assert abs(a.HeadingTrueDeg - 10.0) < 0.01

nav.set_turn_rate(20.0)
time.sleep(0.2)
b = nav.get_attitude()
assert b.HeadingTrueDeg > 13.0

print("NavigationState regression test passed")
