package cp2112

import (
	"fmt"

	log "github.com/sirupsen/logrus"
)

// GpioValue describes the high or low state of a single GPIO pin
// on the CP2112.
type GpioValue uint8

const (
	GpioLow  GpioValue = 0
	GpioHigh GpioValue = 1
)

func (v GpioValue) String() string {
	if v == GpioLow {
		return "Low"
	}
	return "High"
}

// GpioDirection describes the configured direction (in or out) of
// a single GPIO pin on the CP2112.
type GpioDirection uint8

const (
	GpioInput  GpioDirection = 0
	GpioOutput GpioDirection = 1
)

func (v GpioDirection) String() string {
	if v == GpioInput {
		return "Input"
	}
	return "Output"
}

// GpioDrive describes the configured drive topology (push-pull or
// open-drain) of a single GPIO pin on the CP2112.
type GpioDrive uint8

const (
	GpioOpenDrain GpioDrive = 0
	GpioPushPull  GpioDrive = 1
)

func (v GpioDrive) String() string {
	if v == GpioOpenDrain {
		return "OpenDrain"
	}
	return "PushPull"
}

func byteToGpioValues(b byte) [8]GpioValue {
	var vals [8]GpioValue
	for i, v := range byteToInts(b) {
		vals[i] = GpioValue(v)
	}
	return vals
}

func gpioValuesToByte(vals [8]GpioValue) byte {
	var ints [8]int
	for i, v := range vals {
		ints[i] = int(v)
	}
	return intsToByte(ints)
}

func byteToGpioDirections(b byte) [8]GpioDirection {
	var vals [8]GpioDirection
	for i, v := range byteToInts(b) {
		vals[i] = GpioDirection(v)
	}
	return vals
}

func gpioDirectionsToByte(vals [8]GpioDirection) byte {
	var ints [8]int
	for i, v := range vals {
		ints[i] = int(v)
	}
	return intsToByte(ints)
}

func byteToGpioDrives(b byte) [8]GpioDrive {
	var vals [8]GpioDrive
	for i, v := range byteToInts(b) {
		vals[i] = GpioDrive(v)
	}
	return vals
}

func gpioDriveToByte(vals [8]GpioDrive) byte {
	var ints [8]int
	for i, v := range vals {
		ints[i] = int(v)
	}
	return intsToByte(ints)
}

func checkGpioIndex(idx uint) error {
	if idx > 7 {
		return ErrGpioIndexOutOfRange(idx)
	}
	return nil
}

// InvertGpioValue inverts the given GpioValue
func InvertGpioValue(v GpioValue) GpioValue {
	if v == GpioHigh {
		return GpioLow
	}
	return GpioHigh
}

// SetGpioValues sets the values of GPIO pins on the CP2122. The desired value for the pin
// is configured in Latch Value. To drive a "1" on an output pin, the corresponding
// bit should be set to GpioHigh. To drive a "0" on an output pin, the corresponding bit
// should be set to GpioLow.
// The Report sets new values only for output pins that have a true in the corresponding
// bit position in Latch Mask. If the corresponding bit in Latch Mask is set to
// false, a new pin value will not be set, even if the pin is configured as an output pin.
// This Report does not affect any pins that are not configured as outputs.
func (d *CP2112) SetGpioValues(vals [8]GpioValue, mask [8]bool) error {
	errf := errorWrapper("SetGpioValues")
	raw_val := gpioValuesToByte(vals)
	raw_mask := boolsToByte(mask)
	req := []byte{reportIdSetGpioValues, raw_val, raw_mask}
	if n, err := d.dev.SendFeatureReport(req); err != nil {
		return errf(err)
	} else if n != 3 {
		return errf(ErrSentUnexpectedBytes(n))
	}
	log.WithFields(log.Fields{
		"method": "SetGpioValues",
		"values": vals,
		"mask":   mask,
	}).Debugf("Set GPIO values on device.")
	return nil
}

// SetGpioValue sets the GPIO value of a single GPIO pin on the CP2112.
func (d *CP2112) SetGpioValue(idx uint, value GpioValue) error {
	if err := checkGpioIndex(idx); err != nil {
		return err
	}
	var vals [8]GpioValue
	vals[idx] = value
	var mask [8]bool
	mask[idx] = true
	return d.SetGpioValues(vals, mask)
}

// GetGpioValues reads the current values of all the GPIO pins.
// If a pin is configured as a GPIO input pin, the corresponding
// Latch Value bit represents the input value. If a pin is
// configured as a GPIO output pin, the corresponding Latch
// Value bit represents the logic level driven on the pin.
func (d *CP2112) GetGpioValues() ([8]GpioValue, error) {
	errf := errorWrapper("GetGpioValues")
	buf := []byte{reportIdGetGpioValues, 0}
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return [8]GpioValue{}, errf(err)
	} else if n != len(buf) {
		return [8]GpioValue{}, errf(ErrRecvUnexpectedBytes(n))
	}
	vals := byteToGpioValues(buf[1])
	log.WithFields(log.Fields{
		"method": "GetGpioValues",
		"values": vals,
	}).Debugf("Got GPIO values of device.")

	return vals, nil
}

// GetGpioValue gets the current GPIO value of a single GPIO pin on the
// CP2112.
func (d *CP2112) GetGpioValue(idx uint) (GpioValue, error) {
	if err := checkGpioIndex(idx); err != nil {
		return 0, err
	}
	vals, err := d.GetGpioValues()
	if err != nil {
		return 0, err
	}
	return vals[idx], nil
}

// GpioConfiguration describes the configuration of the GPIO pins.
type GpioConfiguration struct {
	Direction         [8]GpioDirection // Pin direction for GPIO pins (in or out)
	Drive             [8]GpioDrive     // Drive topology for GPIO pins (push-pull or open-drain)
	Gpio0TxEnabled    bool             // Indicates if GPIO0 is configured as SMBus Tx indicator.
	Gpio1RxEnabled    bool             // Indicates if GPIO1 is configured as SMBus Rx indicator.
	Gpio7ClockEnabled bool             // Indicates if GPIO7 is configured as clock generator output.
	ClockDivider      byte             // Clock divider setting for GPIO7 clock generator. f(0) = 48 MHz, else f(x) = 48 MHz / (2 * x).
}

func (c *GpioConfiguration) toReport() []byte {
	direction := gpioDirectionsToByte(c.Direction)
	drive := gpioDriveToByte(c.Drive)
	special := byte(0)
	if c.Gpio7ClockEnabled {
		special += (1 << 0)
	}
	if c.Gpio0TxEnabled {
		special += (1 << 1)
	}
	if c.Gpio1RxEnabled {
		special += (1 << 2)
	}
	return []byte{reportIdGpioConfiguration, direction, drive, special, c.ClockDivider}
}

func gpioConfigurationFromReport(buf []byte) (GpioConfiguration, error) {
	errf := errorWrapper("GPIO configuration")
	if len(buf) != 5 {
		return GpioConfiguration{}, errf(ErrUnexpectedReportLength(len(buf)))
	}
	if buf[0] != reportIdGpioConfiguration {
		return GpioConfiguration{}, errf(ErrUnexpectedReportID(buf[0]))
	}
	direction := byteToGpioDirections(buf[1])
	drive := byteToGpioDrives(buf[2])
	special := buf[3]
	clken := ((special >> 0) & 1) == 1
	txen := ((special >> 1) & 1) == 1
	rxen := ((special >> 2) & 1) == 1
	return GpioConfiguration{
		Direction:         direction,
		Drive:             drive,
		Gpio7ClockEnabled: clken,
		Gpio0TxEnabled:    txen,
		Gpio1RxEnabled:    rxen,
		ClockDivider:      buf[4],
	}, nil
}

// GetGpioConfiguration reads the current GPIO configuration from
// the chip.
func (d *CP2112) GetGpioConfiguration() (GpioConfiguration, error) {
	errf := errorWrapper("GetGpioConfiguration")
	buf := []byte{reportIdGpioConfiguration, 0, 0, 0, 0}
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return GpioConfiguration{}, errf(err)
	} else if n != len(buf) {
		return GpioConfiguration{}, errf(ErrRecvUnexpectedBytes(n))
	}
	conf, err := gpioConfigurationFromReport(buf)
	if err != nil {
		return GpioConfiguration{}, err
	}
	log.WithFields(log.Fields{
		"method":        "GetGpioConfiguration",
		"configuration": conf,
	}).Debugf("Got GPIO configuration of device.")
	return conf, nil
}

// SetGpioConfiguration configures the eight GPIO pins
// as input/output and open-drain/push-pull through the Direction
// and Push-Pull fields. GPIO0 corresponds to bit zero, and GPIO7
// corresponds to bit seven. For pins that are configured as an
// input, push-pull mode is ignored. Special is used to enable
// special functionality on GPIO0_TXT, GPIO1_RXT, and GPIO7_CLK.
func (d *CP2112) SetGpioConfiguration(c GpioConfiguration) error {
	errf := errorWrapper("SetGpioConfiguration")
	buf := c.toReport()
	if n, err := d.dev.SendFeatureReport(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
	}
	log.WithFields(log.Fields{
		"method":        "SetGpioConfiguration",
		"configuration": c,
	}).Debugf("Set GPIO configuration of device.")
	return nil
}

// SetGpioDirections sets the GPIO pin direction of all GPIO pins on the
// CP2112.
func (d *CP2112) SetGpioDirections(dirs [8]GpioDirection) error {
	c, err := d.GetGpioConfiguration()
	if err != nil {
		return err
	}
	c.Direction = dirs
	return d.SetGpioConfiguration(c)
}

// SetGpioDirection sets the GPIO pin direction of one GPIO pin on the
// CP2112.
func (d *CP2112) SetGpioDirection(idx uint, dir GpioDirection) error {
	if err := checkGpioIndex(idx); err != nil {
		return nil
	}
	c, err := d.GetGpioConfiguration()
	if err != nil {
		return err
	}
	c.Direction[idx] = dir
	return d.SetGpioConfiguration(c)
}

// EnableRxTxIndicator controls the Rx/Tx indicator function
func (d *CP2112) EnableRxTxIndicator(enableTx, enableRx bool) error {
	errf := errorWrapper("EnableRxTxIndicator")
	c, err := d.GetGpioConfiguration()
	if err != nil {
		return errf(err)
	}
	c.Gpio0TxEnabled = enableTx
	if enableTx {
		c.Direction[0] = GpioOutput
		c.Drive[0] = GpioOpenDrain
	}
	c.Gpio1RxEnabled = enableRx
	if enableRx {
		c.Direction[1] = GpioOutput
		c.Drive[0] = GpioOpenDrain
	}
	err = d.SetGpioConfiguration(c)
	if err != nil {
		return errf(err)
	}
	return nil
}

type ErrInvalidGpio7ClockSpeed int

func (e ErrInvalidGpio7ClockSpeed) Error() string {
	return fmt.Sprintf("invalid clock frequency: %d", e)
}

const (
	clockMax uint = 48_000_000
)

var clockMin = clockMax / (2 * 255)

// CalculateClockFrequency calculates the resulting GPIO7 clock frequency for a given divider.
func CalculateClockFrequency(divider byte) uint {
	if divider == 0 {
		return clockMax
	}
	return clockMax / (2 * uint(divider))
}

// CalculateClockDivider calculates an approximate GPIO7 clock divider setting for a given target
// clock frequency. It also returns the effective clock frequency of that setting.
func CalculateClockDivider(freqHz uint) (byte, uint, error) {
	if freqHz > clockMax || freqHz < clockMin {
		return 0, 0, ErrInvalidGpio7ClockSpeed(freqHz)
	}
	var d byte
	if freqHz == 48_000_000 {
		d = 0
	} else {
		d = byte(24_000_000 / freqHz)
	}
	return d, CalculateClockFrequency(d), nil
}

// EnableGpio7Clock enables the GPIO7 clock output
func (d *CP2112) EnableGpio7Clock(clockDivider byte) error {
	errf := errorWrapper("EnableGpio7Clock")
	c, err := d.GetGpioConfiguration()
	if err != nil {
		return errf(err)
	}
	c.Direction[7] = GpioOutput
	c.Gpio7ClockEnabled = true
	c.ClockDivider = clockDivider
	if err := d.SetGpioConfiguration(c); err != nil {
		return errf(err)
	}
	return nil
}

// DisableGpio7Clock enables the GPIO7 clock output
func (d *CP2112) DisableGpio7Clock() error {
	errf := errorWrapper("DisableGpio7Clock")
	c, err := d.GetGpioConfiguration()
	if err != nil {
		return errf(err)
	}
	c.Gpio7ClockEnabled = false
	if err := d.SetGpioConfiguration(c); err != nil {
		return errf(err)
	}
	return nil
}
