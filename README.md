# Sampler Feedback Streaming

This repository contains a demo of Sampler Feedback Streaming, a technique using sampler feedback to guide loading of tiles of a reserved resource on-demand per-frame. This allows scenes containing 100s of GBs of resources to be drawn using much less physical memory. The scene below uses just ~200MB of a 1GB heap, despite over 350GB of total texture resources.

![Sample screenshot](sampler-feedback-streaming.jpg "Sample screenshot")
Textures derived from [Hubble Images](https://www.nasa.gov/mission_pages/hubble/multimedia/index.html), see the [Hubble Copyright](https://hubblesite.org/copyright)

## License

Copyright 2021 Intel Corporation

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Requirements

The demo requires at least Windows version 19041 and a GPU with Sampler Feedback Support.

Intel Iris Xe Graphics, as can be found in 11th Generation Intel&reg; Core&trade; processors, will run this application as of BETA driver [30.0.100.9667](https://downloadcenter.intel.com/download/30522/Intel-Graphics-BETA-Windows-10-DCH-Drivers)

Note this repository does not contain the textures shown above, which total over 13GB. A link to these textures will hopefully be provided soon. Test textures are provided, as is a mechanism to convert BCx format DDS files.

This repository will be updated When DirectStorage for Windows&reg; becomes available.

## Build Instructions

Download the source. Build the solution file with Visual Studio 2019.

Under x64/Release or x64/Debug find the executable and run.

Note the batch files depend on textures that will be posted at a later date.

## Running

By default, the application starts looking at a single object, "terrain", which allows for exploring sampler feedback streaming.

    c:\SamplerFeedbackStreaming\x64\Release> expanse.exe

![default startup](./readme-images/default-startup.jpg "default startup")

There is a batch file, demo.bat, which starts up in a more interesting state:

    c:\SamplerFeedbackStreaming\x64\Release> demo.bat

![demo batch file](./readme-images/demo-bat.jpg "demo.bat")

## Keyboard controls

There are a lot of keyboard controls - a function of giving many demos:

* `qwe / asd` : strafe left, forward, strafe right / rotate left, back, rotate right
* `z c` : levitate up and down
* `x` : toggles "up lock". When hovering over the "terrain" object, locking the up direction "feels right" with mouse navigation. Otherwise, it should be turned off.
* `v b` : rotate around the look direction (z axis)
* `arrow keys` : rotate left/right, pitch down/up
* `shift` : move faster
* `mouse left-click drag` : rotate view
* `page up` : toggle the min mip map viewer for the "terrain" geometry in the center of the universe
* `page down` : while camera animation is non-zero, toggles fly-through "rollercoaster" vs. fly-around "orbit"
* `space` : toggles camera animation on/off.
* `home` : toggles UI. Hold "shift" while UI is enabled to toggle mini UI mode.
* `end` : toggle overlay of min mip map onto every object
* `insert` : toggles frustum. This behaves a little wonky.
* `esc` : while windowed, exit. while full-screen, return to windowed mode

## JSON configuration files and command lines

For a full list of command line options, pass the command line "?"

Most of the detailed controls for the system can be find in a *json* file. The options in the json have corresponding command lines, e.g.:

json:

    "mediaDir" : "media"

command line:

    -mediadir c:\myMedia

## Misc Notes

Within the source, there is a *TileUpdateManager* library that aspires to be stand-alone. The central object, *TileUpdateManager*, allows for the creation of streaming textures and heaps to contain them. These objects handle all the feedback resource creation, readback, processing, and file/IO.

## Known issues

The demo exhibits texture cracks due to the way feedback is used. Feedback is always read *after* drawing, resulting in loads and evictions corresponding to that frame. That means we never have exactly the texture data we need when we draw (unless no new data is needed). Most of the time this isn't perceptible, but sometimes a fast-moving object enters the view and the system just can't get data there in time resulting in visible artifacts.

The following image shows an exaggerated version of the problem, created by disabling streaming completely then moving the camera:

![Streaming Cracks](streaming-cracks.jpg "Streaming Cracks")

In this case, the hardware sampler is reaching across tile boundaries to perform anisotropic sampling, but encounters tiles that are not physically mapped. D3D12 Reserved Resource tiles that are not physically mapped return black to the sampler. I believe this could be mitigated by "eroding" the min mip map such that there is no more than 1 mip level difference between neighbors. That visual optimization is TBD.

There are also a few known bugs:
* entering full screen in a multi-gpu system moves the window to a monitor attached to the GPU by design. However, if the window starts on a different monitor, it "disappears" on the first maximization. Hit *escape* then maximize again, and it should work fine.
* full-screen while remote desktop is broken *again*. Will likely fix soon.
