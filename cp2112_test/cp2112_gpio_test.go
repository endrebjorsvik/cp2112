package cp2112_test

import (
	"testing"

	"github.com/endrebjorsvik/cp2112"
	"github.com/stretchr/testify/assert"
)

func TestInvertGpioValue(t *testing.T) {
	assert.Equal(t, cp2112.GpioLow, cp2112.InvertGpioValue(cp2112.GpioHigh), "must invert GPIO value")
	assert.Equal(t, cp2112.GpioHigh, cp2112.InvertGpioValue(cp2112.GpioLow), "must invert GPIO value")
}

func TestSetGpioValues(t *testing.T) {
	d := &dummyHid{send: func(buf []byte) (int, error) {
		assert.Equal(t, 3, len(buf), "buffer must have correct length")
		assert.Equal(t, uint8(0x04), buf[0], "report ID must be correct")
		assert.Equal(t, uint8(8), buf[1], "GPIO values byte must be correct")
		assert.Equal(t, uint8(2), buf[2], "GPIO mask byte must be correct")
		return len(buf), nil
	}}
	dev := cp2112.NewCP2112FromHid(d)
	vals := [8]cp2112.GpioValue{
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioHigh,
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioLow,
	}
	mask := [8]bool{false, true, false, false, false, false, false, false}
	err := dev.SetGpioValues(vals, mask)
	assert.NoError(t, err, "function must not fail")
}

func TestSetGpioValueValid(t *testing.T) {
	d := &dummyHid{send: func(buf []byte) (int, error) {
		assert.Equal(t, 3, len(buf), "buffer must have correct length")
		assert.Equal(t, uint8(0x04), buf[0], "report ID must be correct")
		assert.Equal(t, uint8(4), buf[1], "GPIO values byte must be correct")
		assert.Equal(t, uint8(4), buf[2], "GPIO mask byte must be correct")
		return len(buf), nil
	}}
	dev := cp2112.NewCP2112FromHid(d)
	err := dev.SetGpioValue(2, cp2112.GpioHigh)
	assert.NoError(t, err, "function must not fail")
}

func TestSetGpioValueInvalid(t *testing.T) {
	d := &dummyHid{}
	dev := cp2112.NewCP2112FromHid(d)
	err := dev.SetGpioValue(8, cp2112.GpioHigh)
	assert.Error(t, err, "function must fail")
}

func TestGetGpioValues(t *testing.T) {
	d := &dummyHid{send: func(buf []byte) (int, error) {
		assert.Equal(t, 2, len(buf), "buffer must have correct length")
		assert.Equal(t, uint8(0x03), buf[0], "report ID must be correct")
		buf[1] =
		return len(buf), nil
	}}
	dev := cp2112.NewCP2112FromHid(d)
	vals := [8]cp2112.GpioValue{
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioHigh,
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioLow,
		cp2112.GpioLow,
	}
	mask := [8]bool{false, true, false, false, false, false, false, false}
	err := dev.SetGpioValues(vals, mask)
	assert.NoError(t, err, "function must not fail")
}
