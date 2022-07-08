package cp2112_test

import (
	"testing"
	"time"

	"github.com/endrebjorsvik/cp2112"
	"github.com/stretchr/testify/assert"
)

type dummyHid struct{}

func (d *dummyHid) SendFeatureReport(buf []byte) (int, error) {
	return len(buf), nil
}

func (d *dummyHid) GetFeatureReport(buf []byte) (int, error) {
	return len(buf), nil
}

func (d *dummyHid) Write(buf []byte) (int, error) {
	return len(buf), nil
}

func (d *dummyHid) ReadWithTimeout(buf []byte, timeout time.Duration) (int, error) {
	return len(buf), nil
}

func (d *dummyHid) Close() error {
	return nil
}

func TestResetDevice(t *testing.T) {
	dev := cp2112.NewCP2112FromHid(&dummyHid{})
	err := dev.ResetDevice()
	assert.NoError(t, err)
}
