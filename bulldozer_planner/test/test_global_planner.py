import matplotlib.pyplot as plt
from ..bulldozer_planner.sweep_planner import SweepPlanner


# prepare polygons
ox = [0.0, 100.0, 100.0, 0.0, 0.0]
oy = [0.0, 0.0, -50.0, -50.0, 0.0]
resolution = 5.0

# ox = [0.0, 20.0, 50.0, 100.0, 130.0, 40.0, 0.0]
# oy = [0.0, -20.0, 0.0, 30.0, 60.0, 80.0, 0.0]
# resolution = 5.0

# ox = [0.0, 20.0, 50.0, 200.0, 130.0, 40.0, 0.0]
# oy = [0.0, -80.0, 0.0, 30.0, 60.0, 80.0, 0.0]
# resolution = 5.0


def planning_animation(ox, oy, resolution):  # pragma: no cover
    planner = SweepPlanner()
    px, py = planner.planning(ox, oy, resolution)

    for ipx, ipy in zip(px, py):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ox, oy, "-xb")
        plt.plot(px, py, "-r")
        plt.plot(ipx, ipy, "or")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)

    plt.cla()
    plt.plot(ox, oy, "-xb")
    plt.plot(px, py, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)
    plt.close()


planning_animation(ox, oy, resolution)
