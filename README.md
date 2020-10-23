# geonames.gif

An animated gif of 11 million [Geonames](http://www.geonames.org) points.

- All points are loaded in order on a three dimensional z-ordered curve
- The earth is presented with WGS84 flattening factor of 298.25
- Uses [pinhole](https://github.com/tidwall/pinhole) for drawing the points
- Uses [ffmpeg](https://github.com/FFmpeg/FFmpeg) for rendering the final frame

<img src="https://tidwall.s3.amazonaws.com/zorder-12-240.gif">

[Full Size GIF (60+ MB)](https://tidwall.s3.amazonaws.com/zorder-48-512.mp4)  
[Full Size MP4 (5+ MB)](https://tidwall.s3.amazonaws.com/zorder-30-512.mp4)



## Creating the GIF

Download the repo and run:

```
go run main.go
```

It'll take several hours to render on the fastest machine.

