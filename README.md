# (Unofficial) control library for the FY6900 function generator

This is a small and simple control library for the FY6900 function generator. It's
a implementation for the [pylabdevs](https://github.com/tspspi/pylabdevs) ```FunctionGenerator```
class.

# Example usage

## Simple setting of predefined waveforms and parameters

```
import numpy as np

from pyfy6900 import fy6900
from labdevices.functiongenerator import FunctionGeneratorWaveform
from time import sleep

with fy6900.FY6900Serial("COM4", debug = True) as fg:
	print(f"Device identifies as {fg.identify()}")

	fg.set_channel_enabled(0, False)
	fg.set_channel_enabled(1, False)

	fg.set_channel_waveform(0, FunctionGeneratorWaveform.SINE)
	fg.set_channel_frequency(0, 1e3)
	fg.set_channel_offset(0, 2.5)
	fg.set_channel_amplitude(0, 5)
	fg.set_channel_enabled(0, True)

	for frq in np.arange(1, 60e6, 100):
		fg.set_channel_frequency(0, frq)
		print(f"Set new frequency {fg.get_channel_frequency(0)}")

	fg.set_channel_enabled(0, False)
```

## Uploading arbitrary waveform generated using numpy

```
import numpy as np
import matplotlib.pyplot as plt

from pyfy6900 import fy6900

from time import sleep

with fy6900.FY6900Serial("COM3", debug = "True") as fg:
	fg.identify()

	t = np.linspace(0, 2 * np.pi, 8192)
	wv = (np.sin(t) + np.sin(2*t) + np.sin(3*t))

	# Display before upload
	plt.plot(t, wv)
	plt.show()

	# Uploading
	fg.upload_waveform(61, wv, normalize = True)

	# Selecting channel waveform
	fg.set_channel_waveform(0, arbitrary = 61)
```
