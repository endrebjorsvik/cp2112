package cp2112_test

import (
	"errors"
	"fmt"
	"testing"
	"time"

	"github.com/endrebjorsvik/cp2112"
	"github.com/stretchr/testify/assert"
)

var ErrUnintended = errors.New("unintended function")

type dummyHid struct {
	send        func([]byte) (int, error)
	get         func([]byte) (int, error)
	write       func([]byte) (int, error)
	readTimeout func([]byte, time.Duration) (int, error)
	close       func() error
}

func (d *dummyHid) SendFeatureReport(buf []byte) (int, error) {
	if d.send != nil {
		return d.send(buf)
	}
	return 0, fmt.Errorf("%w: SendFeatureReport", ErrUnintended)
}

func (d *dummyHid) GetFeatureReport(buf []byte) (int, error) {
	if d.get != nil {
		return d.get(buf)
	}
	return 0, fmt.Errorf("%w: GetFeatureReport", ErrUnintended)
}

func (d *dummyHid) Write(buf []byte) (int, error) {
	if d.write != nil {
		return d.write(buf)
	}
	return 0, fmt.Errorf("%w: Write", ErrUnintended)
}

func (d *dummyHid) ReadWithTimeout(buf []byte, timeout time.Duration) (int, error) {
	if d.readTimeout != nil {
		return d.readTimeout(buf, timeout)
	}
	return 0, fmt.Errorf("%w: ReadWithTimeout", ErrUnintended)
}

func (d *dummyHid) Close() error {
	if d.close != nil {
		return d.close()
	}
	return fmt.Errorf("%w: Close", ErrUnintended)
}

func TestResetDevice(t *testing.T) {
	d := &dummyHid{send: func(buf []byte) (int, error) {
		assert.Equal(t, 2, len(buf), "buffer must have correct length")
		assert.Equal(t, uint8(0x01), buf[0], "report ID must be correct")
		assert.Equal(t, uint8(0x01), buf[1], "magic number must be correct")
		return len(buf), nil
	}}
	dev := cp2112.NewCP2112FromHid(d)
	err := dev.ResetDevice()
	assert.NoError(t, err)
}

func TestGetVersionInformation(t *testing.T) {
	d := &dummyHid{get: func(buf []byte) (int, error) {
		assert.Equal(t, 3, len(buf), "buffer must have correct length")
		assert.Equal(t, uint8(0x05), buf[0], "report ID must be correct")
		buf[1] = 0x0C
		buf[2] = 0xAB
		return len(buf), nil
	}}
	dev := cp2112.NewCP2112FromHid(d)
	v, err := dev.GetVersionInformation()
	assert.NoError(t, err, "function must not fail")
	assert.Equal(t, uint8(0x0C), v.PartNumber, "part number must be correct")
	assert.Equal(t, uint8(0xAB), v.DeviceVersion, "device version must be correct")
}

func TestGetVersionInformationBad(t *testing.T) {
	d := &dummyHid{get: func(buf []byte) (int, error) {
		assert.Equal(t, 3, len(buf), "buffer must have correct length")
		assert.Equal(t, uint8(0x05), buf[0], "report ID must be correct")
		buf = make([]byte, 2)
		return len(buf), nil
	}}
	dev := cp2112.NewCP2112FromHid(d)
	_, err := dev.GetVersionInformation()
	assert.Error(t, err, "function must fail")
}
