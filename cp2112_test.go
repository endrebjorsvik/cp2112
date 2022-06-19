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
