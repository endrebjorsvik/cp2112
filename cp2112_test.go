package cp2112

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestByteToInts(t *testing.T) {
	stim := make(map[uint8][8]int)
	stim[0] = [8]int{0, 0, 0, 0, 0, 0, 0, 0}
	stim[1] = [8]int{1, 0, 0, 0, 0, 0, 0, 0}
	stim[2] = [8]int{0, 1, 0, 0, 0, 0, 0, 0}
	stim[3] = [8]int{1, 1, 0, 0, 0, 0, 0, 0}
	stim[127] = [8]int{1, 1, 1, 1, 1, 1, 1, 0}
	stim[128] = [8]int{0, 0, 0, 0, 0, 0, 0, 1}
	stim[254] = [8]int{0, 1, 1, 1, 1, 1, 1, 1}
	stim[255] = [8]int{1, 1, 1, 1, 1, 1, 1, 1}

	for b, ex := range stim {
		assert.Equalf(t, ex, byteToInts(b), "should be equal for input %v", b)
	}
}

func TestIntToByte(t *testing.T) {
	stim := make(map[uint8][8]int)
	stim[0] = [8]int{0, 0, 0, 0, 0, 0, 0, 0}
	stim[1] = [8]int{1, 0, 0, 0, 0, 0, 0, 0}
	stim[2] = [8]int{0, 1, 0, 0, 0, 0, 0, 0}
	stim[3] = [8]int{1, 1, 0, 0, 0, 0, 0, 0}
	stim[127] = [8]int{1, 1, 1, 1, 1, 1, 1, 0}
	stim[128] = [8]int{0, 0, 0, 0, 0, 0, 0, 1}
	stim[254] = [8]int{0, 1, 1, 1, 1, 1, 1, 1}
	stim[255] = [8]int{1, 1, 1, 1, 1, 1, 1, 1}

	for ex, bits := range stim {
		assert.Equalf(t, ex, intsToByte(bits), "should be equal for input %v", bits)
	}
}

func TestByteToGpioValues(t *testing.T) {
	stim := make(map[uint8][8]GpioValue)
	stim[0] = [8]GpioValue{0, 0, 0, 0, 0, 0, 0, 0}
	stim[1] = [8]GpioValue{1, 0, 0, 0, 0, 0, 0, 0}
	stim[2] = [8]GpioValue{0, 1, 0, 0, 0, 0, 0, 0}
	stim[3] = [8]GpioValue{1, 1, 0, 0, 0, 0, 0, 0}
	stim[127] = [8]GpioValue{1, 1, 1, 1, 1, 1, 1, 0}
	stim[128] = [8]GpioValue{0, 0, 0, 0, 0, 0, 0, 1}
	stim[254] = [8]GpioValue{0, 1, 1, 1, 1, 1, 1, 1}
	stim[255] = [8]GpioValue{1, 1, 1, 1, 1, 1, 1, 1}

	for b, ex := range stim {
		assert.Equalf(t, ex, byteToGpioValues(b), "should be equal for input %v", b)
	}
}

func TestGpioValuesToByte(t *testing.T) {
	stim := make(map[uint8][8]GpioValue)
	stim[0] = [8]GpioValue{0, 0, 0, 0, 0, 0, 0, 0}
	stim[1] = [8]GpioValue{1, 0, 0, 0, 0, 0, 0, 0}
	stim[2] = [8]GpioValue{0, 1, 0, 0, 0, 0, 0, 0}
	stim[3] = [8]GpioValue{1, 1, 0, 0, 0, 0, 0, 0}
	stim[127] = [8]GpioValue{1, 1, 1, 1, 1, 1, 1, 0}
	stim[128] = [8]GpioValue{0, 0, 0, 0, 0, 0, 0, 1}
	stim[254] = [8]GpioValue{0, 1, 1, 1, 1, 1, 1, 1}
	stim[255] = [8]GpioValue{1, 1, 1, 1, 1, 1, 1, 1}

	for ex, bits := range stim {
		assert.Equalf(t, ex, gpioValuesToByte(bits), "should be equal for input %v", bits)
	}
}

func TestAllGpioValues(t *testing.T) {
	for i := 0; i < 256; i++ {
		b := uint8(i)
		assert.Equal(t, b, gpioValuesToByte(byteToGpioValues(b)), "should be equal")
	}
}

func TestCalculateClockFrequency(t *testing.T) {
	vals := map[byte]uint{
		0:   48_000_000,
		1:   24_000_000,
		2:   12_000_000,
		3:   8_000_000,
		4:   6_000_000,
		255: 94_117,
	}
	for k, v := range vals {
		assert.Equal(t, v, CalculateClockFrequency(k), "should be equal")
	}
}

func TestCalculateClockDividerOk(t *testing.T) {
	type stim struct {
		Freq uint
		Exp  byte
	}
	vals := []stim{
		{48_000_000, 0},
		{47_999_999, 1},

		{24_000_001, 1},
		{24_000_000, 1},
		{23_999_999, 2},

		{12_000_001, 2},
		{12_000_000, 2},
		{11_999_999, 3},

		{8_000_001, 3},
		{8_000_000, 3},
		{7_999_999, 4},

		{6_000_000, 4},
		{94_119, 255},
		{94_118, 255},
		{94_117, 255},
	}
	for _, s := range vals {
		r1, r2, err := CalculateClockDivider(s.Freq)
		assert.NoErrorf(t, err, "%v should not generate an error", s.Freq)
		assert.Equalf(t, s.Exp, r1, "should be equal for %v", s.Freq)
		assert.LessOrEqual(t, r2, s.Freq, "result should be less than or equal to target")
	}
}

func TestCalculateClockDividerClockFreq(t *testing.T) {
	for i := 0; i < 256; i++ {
		d := byte(i)
		f := CalculateClockFrequency(d)
		rd, rf, err := CalculateClockDivider(f)
		assert.NoErrorf(t, err, "frequency %v should not generate an error (divider %v)", f, d)
		assert.Equal(t, d, rd, "dividers should be equal")
		assert.Equal(t, f, rf, "frequencies should be equal")
	}
}
