package cp2112

import (
	"fmt"

	log "github.com/sirupsen/logrus"
	"github.com/sstallion/go-hid"
)

// CP2112 is the primary type for interacting with the SiLabs CP2112
// USB-to-I2C/SMBus controller. The controller also contains a 8 GPIO
// pins that can be controlled through the same interface.
type CP2112 struct {
	dev *hid.Device
}

// DevID is USB HID identification that is used to connect to the correct
// HID device.
type DevID struct {
	Vid    uint16
	Pid    uint16
	Serial string
}

// FindCP2112 finds and returns all compatible CP2112 devices that are
// connected to the system.
func FindCP2112() ([]DevID, error) {
	var devs []DevID
	err := hid.Enumerate(0x10c4, 0xea90,
		func(info *hid.DeviceInfo) error {
			devs = append(devs, DevID{
				Vid: info.VendorID, Pid: info.ProductID, Serial: info.SerialNbr,
			})
			return nil
		})
	return devs, err
}

// NewCP2112 creates and opens a new CP2112 device.
func NewCP2112(vid, pid uint16, serial string) (*CP2112, error) {
	dev, err := hid.Open(vid, pid, serial)
	if err != nil {
		return nil, fmt.Errorf("could not open HID device %d:%d (SN: %s): %w", pid, vid, serial, err)
	}
	return &CP2112{dev: dev}, nil
}

const (
	reportIdResetDevice            byte = 0x01
	reportIdGpioConfiguration      byte = 0x02
	reportIdGetGpioValues          byte = 0x03
	reportIdSetGpioValues          byte = 0x04
	reportIdGetVersionInformation  byte = 0x05
	reportIdSmbusConfiguration     byte = 0x06
	reportIdDataReadRequest        byte = 0x10
	reportIdDataWriteReadRequest   byte = 0x11
	reportIdDataReadForceSend      byte = 0x12
	reportIdDataReadResponse       byte = 0x13
	reportIdDataWrite              byte = 0x14
	reportIdTransferStatusRequest  byte = 0x15
	reportIdTransferStatusResponse byte = 0x16
	reportIdCancelTransfer         byte = 0x17
	reportIdLockByte               byte = 0x20
	reportIdUsbConfiguration       byte = 0x21
	reportIdManufacturingString    byte = 0x22
	reportIdProductString          byte = 0x23
	reportIdSerialString           byte = 0x24
)

// Version holds the device version information. PartNumber indicates the
// device part number. The CP2112 always returns 0x0C. DeviceVersion is the
// version of the device. This value is not programmable over the HID interface.
type Version struct {
	PartNumber    byte
	DeviceVersion byte
}

func (v Version) String() string {
	return fmt.Sprintf("(PartNumber: 0x%02x, Version: 0x%02x)", v.PartNumber, v.DeviceVersion)
}

// Reset Device is used to restart the device from the USB host.
// The device re-enumerates on the USB bus, and all SMBus
// configuration settings are reset to their default values.
// For certain operating systems, such as Windows, initiating a
// device reset and re-enumerating will make the device's handle
// stale. The user application is responsible for handling this
// "surprise disconnect" event. See AN496: CP2112 HID USB-to-SMBus
// API Specification for more information regarding surprise
// disconnects.
func (d *CP2112) ResetDevice() error {
	buf := []byte{reportIdResetDevice, 0x01}
	n_bytes, err := d.dev.SendFeatureReport(buf)
	if err != nil {
		return fmt.Errorf("ResetDevice: %w", err)
	}
	if n_bytes != 2 {
		return fmt.Errorf("ResetDevice sent unexpected number of bytes: %d", n_bytes)
	}
	log.WithFields(log.Fields{
		"method": "ResetDevice",
		"bytes":  n_bytes,
	}).Debugf("Device was reset.")
	return nil
}

// GetVersionInformation reads chip version information from the device.
func (d *CP2112) GetVersionInformation() (Version, error) {
	buf := []byte{reportIdGetVersionInformation, 0, 0}
	n_bytes, err := d.dev.GetFeatureReport(buf)
	if err != nil {
		return Version{}, fmt.Errorf("GetVersionInformation: %w", err)
	}
	if n_bytes != 3 {
		return Version{}, fmt.Errorf("GetVersionInformation received unexpected number of bytes: %d", n_bytes)
	}
	v := Version{
		PartNumber:    buf[1],
		DeviceVersion: buf[2],
	}
	log.WithFields(log.Fields{
		"method":  "GetVersionInformation",
		"bytes":   n_bytes,
		"version": v,
	}).Debugf("Received device version information.")
	return v, nil
}

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

func byteToInts(b byte) [8]int {
	var vals [8]int
	for i := range vals {
		vals[i] = int((b >> i) & 1)
	}
	return vals
}

func intsToByte(vals [8]int) byte {
	var res byte
	for i, v := range vals {
		res += byte(v << i)
	}
	return res
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
		return fmt.Errorf("GPIO index %d is out of range", idx)
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
// The Report sets new values only for output pins that have a "1" in the corresponding
// bit position in Latch Mask. If the corresponding bit in Latch Mask is set to
// "0", a new pin value will not be set, even if the pin is configured as an output pin.
// This Report does not affect any pins that are not configured as outputs.
func (d *CP2112) SetGpioValues(vals [8]GpioValue, mask [8]GpioValue) error {
	raw_val := gpioValuesToByte(vals)
	raw_mask := gpioValuesToByte(mask)
	req := []byte{reportIdSetGpioValues, raw_val, raw_mask}
	n_bytes, err := d.dev.SendFeatureReport(req)
	if err != nil {
		return fmt.Errorf("SetGpioValues: %w", err)
	}
	if n_bytes != 3 {
		return fmt.Errorf("SetGpioValues sent unexpected number of bytes: %d", n_bytes)
	}
	log.WithFields(log.Fields{
		"method": "SetGpioValues",
		"bytes":  n_bytes,
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
	var mask [8]GpioValue
	mask[idx] = GpioHigh
	return d.SetGpioValues(vals, mask)
}

// GetGpioValues reads the current values of all the GPIO pins.
// If a pin is configured as a GPIO input pin, the corresponding
// Latch Value bit represents the input value. If a pin is
// configured as a GPIO output pin, the corresponding Latch
// Value bit represents the logic level driven on the pin.
func (d *CP2112) GetGpioValues() ([8]GpioValue, error) {
	buf := []byte{reportIdGetGpioValues, 0}
	n_bytes, err := d.dev.GetFeatureReport(buf)
	if err != nil {
		return [8]GpioValue{}, fmt.Errorf("GetGpioValues: %w", err)
	}
	if n_bytes != 2 {
		return [8]GpioValue{}, fmt.Errorf("GetGpioValues received unexpected number of bytes: %d", n_bytes)
	}
	vals := byteToGpioValues(buf[1])
	log.WithFields(log.Fields{
		"method": "GetGpioValues",
		"bytes":  n_bytes,
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
	Direction         [8]GpioDirection
	Drive             [8]GpioDrive
	Gpio0TxEnabled    bool
	Gpio1RxEnabled    bool
	Gpio7ClockEnabled bool
	ClockDivider      byte
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

func configurationFromReport(buf []byte) (GpioConfiguration, error) {
	if len(buf) != 5 {
		return GpioConfiguration{}, fmt.Errorf("unexpected GPIO configuration report length: %d", len(buf))
	}
	if buf[0] != reportIdGpioConfiguration {
		return GpioConfiguration{}, fmt.Errorf("unexpected GPIO configuration report ID: %d", buf[0])
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
	buf := []byte{reportIdGpioConfiguration, 0, 0, 0, 0}
	n_bytes, err := d.dev.GetFeatureReport(buf)
	if err != nil {
		return GpioConfiguration{}, fmt.Errorf("GetGpioConfiguration: %w", err)
	}
	if n_bytes != 5 {
		return GpioConfiguration{}, fmt.Errorf("GetGpioConfiguration received unexpected number of bytes: %d", n_bytes)
	}
	conf, err := configurationFromReport(buf)
	if err != nil {
		return GpioConfiguration{}, err
	}
	log.WithFields(log.Fields{
		"method":        "GetGpioConfiguration",
		"bytes":         n_bytes,
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
	buf := c.toReport()
	n_bytes, err := d.dev.SendFeatureReport(buf)
	if err != nil {
		return fmt.Errorf("SetGpioConfiguration: %w", err)
	}
	if n_bytes != 5 {
		return fmt.Errorf("SetGpioConfiguration sent unexpected number of bytes: %d", n_bytes)
	}
	log.WithFields(log.Fields{
		"method":        "SetGpioConfiguration",
		"bytes":         n_bytes,
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
