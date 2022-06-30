package cp2112

import (
	"testing"
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
		r := byteToInts(b)
		if r != ex {
			t.Errorf("byteToInts(%d) = %v, expected: %v.", b, r, ex)
		}
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
		r := intsToByte(bits)
		if r != ex {
			t.Errorf("intsToByte(%d) = %v, expected: %v.", bits, r, ex)
		}
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
		r := byteToGpioValues(b)
		if r != ex {
			t.Errorf("byteToGpioValues(%d) = %v, expected: %v.", b, r, ex)
		}
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
		r := gpioValuesToByte(bits)
		if r != ex {
			t.Errorf("gpioValuesToByte(%d) = %v, expected: %v.", bits, r, ex)
		}
	}
}

func TestAllGpioValues(t *testing.T) {
	for i := 0; i < 256; i++ {
		b := uint8(i)
		r := gpioValuesToByte(byteToGpioValues(b))
		if r != b {
			t.Errorf("gpioValuesToByte(byteToGpioValues(%d)) = %d.", b, r)
		}
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
		d := uint8(k)
		r := CalculateClockFrequency(d)
		if r != v {
			t.Errorf("CalculateClockFrequency(%v) = %v.", d, r)
		}
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
	for i, s := range vals {
		r1, r2, err := CalculateClockDivider(s.Freq)
		if err != nil {
			t.Error(err)
		}
		if r1 != s.Exp || r2 > s.Freq {
			t.Errorf("%v: CalculateClockDivider(%v) = %v, %v.", i, s.Freq, r1, r2)
		}
	}
}

func TestCalculateClockDividerClockFreq(t *testing.T) {
	for i := 0; i < 256; i++ {
		d := byte(i)
		f := CalculateClockFrequency(d)
		rd, rf, err := CalculateClockDivider(f)
		if err != nil {
			t.Error(err)
		}
		if rd != d || f != rf {
			t.Errorf("%v: CalculateClockDivider(CalculateClockFrequency(%v)) = %v, %v. Should have %v", i, d, rd, rf, f)
		}
	}
}
