# Samples from a matplotlib colormap and prints a constant string to stdout, which can then be embedded in a C++ program

import numpy as np
import matplotlib.cm
import matplotlib.pyplot as plt
import colorsys

nValues = 500

# Custom rainbow palette with smooth color transitions
def custom_palette(t):
    """
    Custom rainbow palette with smooth color transitions:
    red → orange → yellow → green → cyan → blue → blue-violet → magenta
    """
    if t < 0.143:
        # red → orange
        factor = t / 0.143
        return (1, 0 + factor * 0.5, 0)
    elif t < 0.286:
        # orange → yellow
        factor = (t - 0.143) / 0.143
        return (1, 0.5 + factor * 0.5, 0)
    elif t < 0.429:
        # yellow → green
        factor = (t - 0.286) / 0.143
        return (1 - factor, 1, 0)
    elif t < 0.571:
        # green → cyan
        factor = (t - 0.429) / 0.142
        return (0, 1, factor)
    elif t < 0.714:
        # cyan → blue
        factor = (t - 0.571) / 0.143
        return (0, 1 - factor, 1)
    elif t < 0.857:
        # blue → blue-violet
        factor = (t - 0.714) / 0.143
        return (factor * 0.5, 0, 1)
    elif t < 1.0:
        # blue-violet → magenta
        factor = (t - 0.857) / 0.143
        return (0.5 + factor * 0.5, 0, 1)
    else:
        # magenta → light magenta for values > 1.0
        factor = min((t - 1.0) / 0.143, 1.0)
        return (1, factor * 0.5, 1)


cmapName = "custom_rainbow"

print("static const Colormap CM_" + cmapName.upper() + " = {")
print('    "' + cmapName + '",')

dataStr = "{"
for i in range(nValues):
    floatInd = float(i) / (nValues - 1)
    color = custom_palette(floatInd)
    dataStr += "{" + str(color[0]) + "," + str(color[1]) + "," + str(color[2]) + "},"

dataStr += "}"
print(dataStr)

print("};")

# print("};")
