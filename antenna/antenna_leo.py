from antennas import (
    generate_feedline as feedline, generate_patch, generate_pcb, translate)

zones = []
add = zones.append

# PCB constants
εr = 3.66
h = 0.254e-3

# Patch dimensions
Lp = 3.165e-3
Wp = 4.000e-3
x0 = 1.055e-3
y0 = 1.000e-3
patch = generate_patch(Wp, Lp, y0, x0)

# Microstrip widths
w35 = 0.917e-3
w50 = 0.500e-3
w70 = 0.278e-3

# Full wavelength in microstrips of different impedances
λ35 = 7.198e-3
λ50 = 7.430e-3
λ70 = 7.711e-3

# Other free geometry
colsep = 0
patchsep = 1.500e-3

# Start from a feed point at (0, 0)

# Mirror for positive and negative y
for ydir in (+1,):

    # Mirror for positive and negative x
    for xdir in (+1, -1):

        # Transformer from 100 (half splitpoint) to 50 (feed outwards)
        add(feedline([(0, ydir*colsep), (xdir*λ70/4, ydir*colsep)], w70, h))

        # Transformer from 50 to 25 (split into two patches)
        add(feedline([(xdir*λ70/4, ydir*colsep),
                      (xdir*λ70/4 + xdir*λ35/4, ydir*colsep)], w35, h))

        # Feedline down to first patch
        add(feedline([(xdir*λ70/4 + xdir*λ35/4, ydir*colsep),
                      (xdir*λ70/4 + xdir*λ35/4, ydir*colsep + patchsep + x0)],
                     w50, h))

        # Feedline around to second patch
        add(feedline([(xdir*λ70/4 + xdir*λ35/4, ydir*colsep),
                      (xdir*λ70/4 + xdir*λ35/4 + xdir*λ50, ydir*colsep),
                      (xdir*λ70/4 + xdir*λ35/4 + xdir*λ50,
                       ydir*colsep + patchsep + x0)],
                     w50, h))

        # First patch
        add(translate(patch, xdir*λ70/4 + xdir*λ35/4,
                      ydir * colsep + patchsep + Lp/2))

        # Second patch
        add(translate(patch, xdir*λ70/4 + xdir*λ35/4 + xdir*λ50,
                      ydir * colsep + patchsep + Lp/2))

pcb_centrefed = generate_pcb([(0, 0)], [], zones, [])

if __name__ == "__main__":
    with open("antenna_leo.kicad_pcb", "w") as f:
        f.write(pcb_centrefed)
