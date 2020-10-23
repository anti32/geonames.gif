package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/png"
	"io"
	"io/ioutil"
	"math"
	"math/rand"
	"os"
	"os/exec"
	"reflect"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
	"unsafe"

	"github.com/tidwall/cell.go/xyz"
	"github.com/tidwall/pinhole"
)

const degToRad = math.Pi / 180
const radToDeg = 180 / math.Pi

func lonLatElevToXYZ(lle [3]float64) (xyz [3]float64) {
	// see http://www.mathworks.de/help/toolbox/aeroblks/llatoecefposition.html
	const radius = 6378137.0               // Radius of the Earth (in meters)
	const flattening = 1.0 / 298.257223563 // Flattening factor WGS84 Model
	const ff2 = (1.0 - flattening) * (1.0 - flattening)
	const degToRad = math.Pi / 180

	sinLon, cosLon := math.Sincos(lle[0] * degToRad)
	sinLat, cosLat := math.Sincos(lle[1] * degToRad)
	ele := lle[2]
	c := 1.0 / math.Sqrt((cosLat*cosLat)+(ff2*sinLat*sinLat))
	x := (radius*c + ele) * cosLat * cosLon
	y := (radius*c + ele) * cosLat * sinLon
	z := (radius*c*ff2 + ele) * sinLat

	return [3]float64{x, z, y} // notice the y and z are switch for rotation
}

func xyzToPinhole(xyz [3]float64) [3]float64 {
	const maxCoord = 0.99999999999999988897769753748434595763683319091796875
	const radius = 6378137.0 // Radius of the Earth (in meters)

	xyz[0] = math.Min(math.Max(xyz[0]+radius, 0), radius*2) / (radius * 2)
	xyz[1] = math.Min(math.Max(xyz[1]+radius, 0), radius*2) / (radius * 2)
	xyz[2] = math.Min(math.Max(xyz[2]+radius, 0), radius*2) / (radius * 2)

	xyz[0] = math.Min(xyz[0], maxCoord) - 0.5
	xyz[1] = math.Min(xyz[1], maxCoord) - 0.5
	xyz[2] = math.Min(xyz[2], maxCoord) - 0.5

	return xyz
}

// xyzToGrid convert the xyz from lonLatElevToXYZ to a flat grid coordinates
// where each output coord is 0-max(uint64)
func xyzToGrid(xyz [3]float64) [3]uint64 {
	xyz = xyzToPinhole(xyz)

	return [3]uint64{
		uint64((xyz[0] + 0.5) * (1 << 64)),
		uint64((xyz[1] + 0.5) * (1 << 64)),
		uint64((xyz[2] + 0.5) * (1 << 64)),
	}
}

type point struct {
	x, y, z float64
}

type frame struct {
	num       int
	nframes   int
	totalSecs float64
	spinSecs  float64
	fadeSecs  float64
	fps       float64
	spinRate  float64
	width     int
	height    int
	points    []point
	showClock bool
	parts     int
	part      int
	ready     *readyMap
	skip      int
}

type readyMap struct {
	mu     sync.Mutex
	frames map[int]int
}

func renderPartFrame(f frame) {
	finalPath := fmt.Sprintf("frames/%d.png", f.num)
	if _, err := os.Stat(finalPath); err == nil {
		return
	}
	partPath := fmt.Sprintf("frames/%d-%d.png", f.num, f.part)
	if _, err := os.Stat(partPath); err != nil {

		pointStopPos := 1 - (f.spinSecs / f.totalSecs)
		pointPos := float64(f.num) / (float64(f.nframes) * pointStopPos)
		endPoint := int(float64(len(f.points))*pointPos) + 1
		if endPoint > len(f.points) {
			endPoint = len(f.points)
		}
		pos := float64(f.num) / float64(f.nframes)
		fadePos := 1 - (f.fadeSecs / f.totalSecs)
		rotatePos := f.spinRate * (f.totalSecs * pos)
		p := pinhole.New()
		j := 0
		for i := 0; i < endPoint; i++ {
			pt := f.points[i]
			if i%f.parts == f.part {
				if j%(f.skip+1) == 0 {
					p.DrawDot(pt.x, pt.y, pt.z, 0.0002*float64((f.skip/100)+1))
				}
				j++
			}
		}
		p.Rotate(0, math.Pi*2*rotatePos, 0)
		if f.showClock {
			p.DrawString(0, 0, 0, fmt.Sprintf("%.1f", pos*f.totalSecs))
		}
		if pos >= fadePos {
			fade := (pos - fadePos) / (f.fadeSecs / f.totalSecs)
			p.Colorize(color.RGBA{0, 0, 0, uint8(255.0 * (1 - fade))})
		}
		if f.parts == 1 {
			must(nil, p.SavePNG(finalPath, f.width, f.height, nil))
			// fmt.Fprintf(os.Stderr, "combine %d\n", f.num)
		} else {
			opts := *pinhole.DefaultImageOptions
			opts.BGColor = color.Transparent
			must(nil, p.SavePNG(partPath+".tmp", f.width, f.height, &opts))
			must(nil, os.Rename(partPath+".tmp", partPath))
		}
	}
	if f.parts != 1 {
		f.ready.mu.Lock()
		f.ready.frames[f.num] = f.ready.frames[f.num] + 1
		combine := f.ready.frames[f.num] == f.parts
		f.ready.mu.Unlock()

		if combine {
			var dst draw.Image
			for i := 0; i < f.parts; i++ {
				path := fmt.Sprintf("frames/%d-%d.png", f.num, i)
				imgf := must(os.Open(path)).(*os.File)
				src := must(png.Decode(imgf)).(image.Image)
				imgf.Close()
				if dst == nil {
					dst = image.NewRGBA(src.Bounds())
					draw.Draw(dst, dst.Bounds(), &image.Uniform{color.White},
						image.ZP, draw.Src)
				}
				draw.Draw(dst, dst.Bounds(), src, image.ZP, draw.Over)
			}
			imgf := must(os.Create(finalPath + ".tmp")).(*os.File)
			must(nil, png.Encode(imgf, dst))
			imgf.Close()
			must(nil, os.Rename(finalPath+".tmp", finalPath))
			for i := 0; i < f.parts; i++ {
				path := fmt.Sprintf("frames/%d-%d.png", f.num, i)
				must(nil, os.Remove(path))
			}
			fmt.Fprintf(os.Stderr, "combine %d\n", f.num)
		}
	}
}

func render(ch chan frame, wg *sync.WaitGroup) {
	defer wg.Done()
	for f := range ch {
		renderPartFrame(f)
	}
}

func main() {
	var forwards, backwards, random bool
	var only int
	var low bool
	flag.BoolVar(&forwards, "forwards", false, "")
	flag.BoolVar(&backwards, "backwards", false, "")
	flag.BoolVar(&random, "random", false, "")
	flag.IntVar(&only, "only", -1, "")
	flag.BoolVar(&low, "low", false, "low quality")
	flag.Parse()

	downloadGeonames()
	generateCurves()

	totalSecs := 30.0  // total animation time
	spinSecs := 25.0   // total final spin time
	fadeSecs := 1.0    // total fade time
	spinRate := 0.0625 // rotations per second
	fps := 96.0
	width := 1024
	height := 1024
	showClock := false
	which := "zorder"
	parts := 10
	ready := &readyMap{frames: make(map[int]int)}
	skip := 0

	if low {
		fps = 15
		skip = 1000
		parts = 1
	}

	nframes := int(totalSecs * fps)

	// os.RemoveAll("frames")
	os.MkdirAll("frames", 0777)
	// reading points
	fmt.Fprintf(os.Stderr, "Reading points...\n")
	data := must(ioutil.ReadFile(which + ".bin")).([]byte)
	points := *((*[]point)(unsafe.Pointer(&reflect.SliceHeader{
		Len:  len(data) / 24,
		Cap:  len(data) / 24,
		Data: uintptr(unsafe.Pointer(&data[0])),
	})))
	nrenderers := runtime.NumCPU()
	ch := make(chan frame)
	var wg sync.WaitGroup
	wg.Add(nrenderers)
	for i := 0; i < nrenderers; i++ {
		go render(ch, &wg)
	}
	rand.Seed(time.Now().UnixNano())

	var perm []int
	if only != -1 {
		for i := only * parts; i < only*parts+parts; i++ {
			perm = append(perm, i)
		}
	} else if forwards {
		for i := 0; i < nframes*parts; i++ {
			perm = append(perm, i)
		}
	} else if backwards {
		for i := 0; i < nframes*parts; i++ {
			perm = append(perm, nframes*parts-1-i)
		}
	} else {
		perm = rand.Perm(nframes * parts)
	}

	for i := 0; i < len(perm); i++ {
		num := perm[i]
		ch <- frame{
			num:       num / parts,
			nframes:   nframes,
			totalSecs: totalSecs,
			fps:       fps,
			spinSecs:  spinSecs,
			fadeSecs:  fadeSecs,
			points:    points,
			width:     width,
			height:    height,
			spinRate:  spinRate,
			showClock: showClock,
			parts:     parts,
			part:      num % parts,
			ready:     ready,
			skip:      skip,
		}
		fmt.Fprintf(os.Stderr, "\rGenerating frames... %.4f%% ",
			float64(i)/float64(nframes*parts)*100)
	}
	fmt.Fprintf(os.Stderr, "\rGenerating frames... 100%%       \n")

	close(ch)
	wg.Wait()
	if only != -1 {
		return
	}

	makePalette()
	run("ffmpeg", "-y", "-hide_banner",
		"-r", fmt.Sprint(fps), "-i", "frames/%d.png",
		"-i", "frames/palette.png",
		"-filter_complex",
		fmt.Sprintf("fps=48,scale=%d:-1:flags=lanczos[x];[x][1:v]paletteuse",
			int(512)),
		which+".gif")
}

func makePalette() {
	p := pinhole.New()
	p.DrawCube(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5)
	// p.Colorize(color.White)
	// opts := *pinhole.DefaultImageOptions
	// opts.BGColor = color.RGBA{255, 89, 94, 255}
	p.SavePNG("frames/palette-intermediate.png", 500, 500, nil) //&opts)

	run("ffmpeg", "-y", "-loglevel", "panic",
		"-i", "frames/palette-intermediate.png",
		"-vf", "fps=60,scale=500:-1:flags=lanczos,palettegen",
		"frames/palette.png")
}

func downloadGeonames() {
	if _, err := os.Stat("allCountries.txt"); err == nil {
		return
	}
	run("wget", "--progress=bar:force", "-c",
		"http://download.geonames.org/export/dump/allCountries.zip")
	run("unzip", "-o", "allCountries.zip", "-d", "extract")
	run("mv", "extract/allCountries.txt", ".")
	run("rm", "-rf", "extract", "allCountries.zip")
}

func generateCurves() {
	_, err1 := os.Stat("hilbert.bin")
	_, err2 := os.Stat("zorder.bin")
	if err1 == nil && err2 == nil {
		return
	}
	f := must(os.Open("allCountries.txt")).(*os.File)
	total := int(must(f.Stat()).(os.FileInfo).Size())
	rd := bufio.NewReader(f)
	type point struct {
		hilbert uint64
		zorder  xyz.Cell
		xyz     [3]float64
	}
	var points []point
	var read int
	for i := 0; ; i++ {
		line, err := rd.ReadBytes('\n')
		if err != nil {
			if err == io.EOF {
				break
			}
			panic(err)
		}
		read += len(line)
		parts := strings.Split(string(line), "\t")
		lat, _ := strconv.ParseFloat(parts[4], 64)
		lon, _ := strconv.ParseFloat(parts[5], 64)
		points = append(points, point{
			zorder:  zorderEncode(lat, lon),
			hilbert: hilbertEncode(lat, lon),
		})
		if i%7531 == 0 || read >= total {
			fmt.Fprintf(os.Stderr, "\rGenerating curves... %.1f%% ",
				float64(read)/float64(total)*100)
			if read >= total {
				fmt.Fprintf(os.Stderr, "\n")
			}
		}
	}

	fmt.Fprintf(os.Stderr, "\rWriting hilbert.bin...\n")
	sort.SliceStable(points, func(i, j int) bool {
		return points[i].hilbert < points[j].hilbert
	})
	var fbuf [8]byte
	wr := bufio.NewWriter(must(os.Create("hilbert-work.bin")).(*os.File))
	for _, pt := range points {
		binary.LittleEndian.PutUint64(fbuf[:], math.Float64bits(pt.xyz[0]))
		must(wr.Write(fbuf[:]))
		binary.LittleEndian.PutUint64(fbuf[:], math.Float64bits(pt.xyz[1]))
		must(wr.Write(fbuf[:]))
		binary.LittleEndian.PutUint64(fbuf[:], math.Float64bits(pt.xyz[2]))
		must(wr.Write(fbuf[:]))
	}
	must(nil, wr.Flush())

	fmt.Fprintf(os.Stderr, "\rWriting zorder.bin...\n")
	sort.SliceStable(points, func(i, j int) bool {
		if points[i].zorder.Hi < points[j].zorder.Hi {
			return true
		}
		if points[i].zorder.Hi > points[j].zorder.Hi {
			return false
		}
		return points[i].zorder.Lo < points[j].zorder.Hi
	})
	wr = bufio.NewWriter(must(os.Create("zorder-work.bin")).(*os.File))
	for _, pt := range points {
		binary.LittleEndian.PutUint64(fbuf[:], math.Float64bits(pt.xyz[0]))
		must(wr.Write(fbuf[:]))
		binary.LittleEndian.PutUint64(fbuf[:], math.Float64bits(pt.xyz[1]))
		must(wr.Write(fbuf[:]))
		binary.LittleEndian.PutUint64(fbuf[:], math.Float64bits(pt.xyz[2]))
		must(wr.Write(fbuf[:]))
	}
	must(nil, wr.Flush())

	run("mv", "hilbert-work.bin", "hilbert.bin")
	run("mv", "zorder-work.bin", "zorder.bin")
}

func zorderEncode(lat, lon float64) xyz.Cell {
	xyzPt := lonLatElevToXYZ([3]float64{lon, lat, 0})
	grid := xyzToGrid(xyzPt)
	return xyz.Encode(
		float64(grid[0])/math.MaxUint64,
		float64(grid[1])/math.MaxUint64,
		float64(grid[2])/math.MaxUint64,
	)
}

func hilbertEncode(lat, lon float64) uint64 {
	// no hilbert encoding atm :( SORRY!!! lol
	return 0
}

func run(name string, args ...string) {
	cmd := exec.Command(name, args...)
	stdout, _ := cmd.StdoutPipe()
	go io.Copy(os.Stdout, stdout)
	stderr, _ := cmd.StderrPipe()
	go io.Copy(os.Stdout, stderr)
	stdin, _ := cmd.StdinPipe()
	defer stdin.Close()
	must(nil, cmd.Start())
	must(nil, cmd.Wait())
}

func must(v interface{}, err error) interface{} {
	if err != nil {
		panic(err)
	}
	return v
}
