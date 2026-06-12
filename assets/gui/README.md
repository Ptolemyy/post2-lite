Earth GUI texture
=================

`earth_blue_marble_2048.bmp` is derived from NASA's Blue Marble Next Generation
global surface mosaic at 2048x1024 resolution.

Source:
https://svs.gsfc.nasa.gov/2915/

Conversion:
- Downloaded `bluemarble-2048.png` from the NASA Scientific Visualization
  Studio page above.
- Converted to a 24-bit BMP so the Win32 GUI can embed it as a resource and
  decode it without adding an image library dependency.
