# HSV to RGB

Tanner Weber 2026

This program lets the microbit v2 translate HSV from a potentiometer into RGB
to manipulate an LED.

![Video demo in ./VIDEO.mkv](./VIDEO.mkv)

# 🚀 Build and Run

```probe-rs-tools``` is needed

```cargo embed --release```

# 📖 Writeup

My implementation gets new RGB values each frame and uses that to calculate
a schedule for the delays between each LED pin being turned off for PWM. I used
an array of tuples containing a pin color and delay that gets sorted.

# License

Copyright (C) 2026 Tanner Weber

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
