package cp2112

import (
	"encoding/binary"
	"fmt"
	"time"

	log "github.com/sirupsen/logrus"
	"github.com/sstallion/go-hid"
)

// CP2112 is the primary type for interacting with the SiLabs CP2112
// USB-to-I2C/SMBus controller. The controller also contains a 8 GPIO
// pins that can be controlled through the same interface.
type CP2112 struct {
	dev *hid.Device // Open device handle
}

// DevID is USB HID identification that is used to connect to the correct
// HID device.
type DevID struct {
	Vid    uint16 // Device Vendor ID
	Pid    uint16 // Device Product ID
	Serial string // Device serial number
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
	log.WithFields(log.Fields{
		"vid":    vid,
		"pid":    pid,
		"serial": serial,
	}).Debugf("Opened CP2112 device.")
	return &CP2112{dev: dev}, nil
}

func (c *CP2112) Close() error {
	log.Debugf("Closing CP2112 device.")
	return c.dev.Close()
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

	smbusClockSpeedHzMin uint32 = 10_000
	smbusClockSpeedHzMax uint32 = 400_000
)

// Version holds the device version information. PartNumber indicates the
// device part number. The CP2112 always returns 0x0C. DeviceVersion is the
// version of the device. This value is not programmable over the HID interface.
type Version struct {
	PartNumber    byte // Device part number. Always 0x0C.
	DeviceVersion byte // Device version number.
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
	if n, err := d.dev.SendFeatureReport(buf); err != nil {
		return fmt.Errorf("ResetDevice: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("ResetDevice sent unexpected number of bytes: %d", n)
	}
	log.WithFields(log.Fields{
		"method": "ResetDevice",
	}).Debugf("Device was reset.")
	return nil
}

// GetVersionInformation reads chip version information from the device.
func (d *CP2112) GetVersionInformation() (Version, error) {
	buf := []byte{reportIdGetVersionInformation, 0, 0}
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return Version{}, fmt.Errorf("GetVersionInformation: %w", err)
	} else if n != 3 {
		return Version{}, fmt.Errorf("GetVersionInformation received unexpected number of bytes: %d", n)
	}
	v := Version{
		PartNumber:    buf[1],
		DeviceVersion: buf[2],
	}
	log.WithFields(log.Fields{
		"method":  "GetVersionInformation",
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
	if n, err := d.dev.SendFeatureReport(req); err != nil {
		return fmt.Errorf("SetGpioValues: %w", err)
	} else if n != 3 {
		return fmt.Errorf("SetGpioValues sent unexpected number of bytes: %d", n)
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
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return [8]GpioValue{}, fmt.Errorf("GetGpioValues: %w", err)
	} else if n != len(buf) {
		return [8]GpioValue{}, fmt.Errorf("GetGpioValues received unexpected number of bytes: %d", n)
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
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return GpioConfiguration{}, fmt.Errorf("GetGpioConfiguration: %w", err)
	} else if n != len(buf) {
		return GpioConfiguration{}, fmt.Errorf("GetGpioConfiguration received unexpected number of bytes: %d", n)
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
	buf := c.toReport()
	if n, err := d.dev.SendFeatureReport(buf); err != nil {
		return fmt.Errorf("SetGpioConfiguration: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("SetGpioConfiguration sent unexpected number of bytes: %d", n)
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
	c, err := d.GetGpioConfiguration()
	if err != nil {
		return fmt.Errorf("EnableRxTxIndicator: %w", err)
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
		return fmt.Errorf("EnableRxTxIndicator: %w", err)
	}
	return nil
}

// SmbusConfiguration contains configuration for the SMBus/I2C interface.
type SmbusConfiguration struct {
	ClockSpeedHz  uint32        // SMBus/I2C clock speed given in hertz.
	DeviceAddress byte          // 7-bit device address of the CP2112. Master device address. Only ACKed, but can not communicate.
	AutoSendRead  bool          // Automatically send read response interrupt reports to the host after a read transfer is initiated
	WriteTimeout  time.Duration // Time limit before the CP2112 automatically cancels a transfer that has been initiated. Zero is infinite.
	ReadTimeout   time.Duration // Time limit before the CP2112 automatically cancels a transfer that has been initiated. Zero is infinite.
	SclLowTimeout bool          // Resets the SMBus if the SCL line is held low for more than 25 ms.
	RetryTimes    uint16        // Number of attempts that the CP2112 attempts to complete a transfer before terminating the transfer. Values larger than 1000 are ignored.
}

func (c SmbusConfiguration) toReport() ([]byte, error) {
	if (c.ClockSpeedHz < smbusClockSpeedHzMin) || (c.ClockSpeedHz > smbusClockSpeedHzMax) {
		return nil, fmt.Errorf("invalid SMBus/I2C clock speed: %d Hz", c.ClockSpeedHz)
	}
	buf := make([]byte, 14)
	buf[0] = reportIdSmbusConfiguration
	binary.BigEndian.PutUint32(buf[1:5], c.ClockSpeedHz)
	buf[5] = c.DeviceAddress
	if c.AutoSendRead {
		buf[6] = 1
	} else {
		buf[6] = 0
	}
	binary.BigEndian.PutUint16(buf[7:9], uint16(c.WriteTimeout.Milliseconds()))
	binary.BigEndian.PutUint16(buf[9:11], uint16(c.ReadTimeout.Milliseconds()))
	if c.SclLowTimeout {
		buf[11] = 1
	} else {
		buf[11] = 0
	}
	binary.BigEndian.PutUint16(buf[12:14], c.RetryTimes)
	return buf, nil
}

func smbusConfigurationFromReport(buf []byte) (SmbusConfiguration, error) {
	if len(buf) != 14 {
		return SmbusConfiguration{}, fmt.Errorf("unexpected SMBus configuration report length: %d", len(buf))
	}
	if buf[0] != reportIdSmbusConfiguration {
		return SmbusConfiguration{}, fmt.Errorf("unexpected SMBus configuration report ID: %d", buf[0])
	}
	return SmbusConfiguration{
		ClockSpeedHz:  binary.BigEndian.Uint32(buf[1:5]),
		DeviceAddress: buf[5],
		AutoSendRead:  buf[6] == 1,
		WriteTimeout:  time.Duration(binary.BigEndian.Uint16(buf[7:9])) * time.Millisecond,
		ReadTimeout:   time.Duration(binary.BigEndian.Uint16(buf[9:11])) * time.Millisecond,
		SclLowTimeout: buf[11] == 1,
		RetryTimes:    binary.BigEndian.Uint16(buf[12:14]),
	}, nil
}

// GetSmbusConfiguration reads the SMBus/I2C configuration from CP2112.
func (d *CP2112) GetSmbusConfiguration() (SmbusConfiguration, error) {
	buf := make([]byte, 14)
	buf[0] = reportIdSmbusConfiguration
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return SmbusConfiguration{}, fmt.Errorf("GetSmbusConfiguration: %w", err)
	} else if n != len(buf) {
		return SmbusConfiguration{}, fmt.Errorf("GetSmbusConfiguration received unexpected number of bytes: %d", n)
	}
	return smbusConfigurationFromReport(buf)
}

// SetSmbusConfiguration writes the given SMBus/I2C configuration to CP2112.
func (d *CP2112) SetSmbusConfiguration(c SmbusConfiguration) error {
	buf, err := c.toReport()
	if err != nil {
		return fmt.Errorf("SetSmbusConfiguration: %w", err)
	}
	if n, err := d.dev.SendFeatureReport(buf); err != nil {
		return fmt.Errorf("SetSmbusConfiguration: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("SetSmbusConfiguration sent unexpected number of bytes: %d", n)
	}
	return nil
}

// GetSmbusClockSpeedHz gets the currently configured SMBus/I2C clock frequency in hertz.
func (d *CP2112) GetSmbusClockSpeedHz() (uint32, error) {
	c, err := d.GetSmbusConfiguration()
	if err != nil {
		return 0, err
	}
	return c.ClockSpeedHz, nil
}

// SetSmbusClockSpeedHz sets the currently configured SMBus/I2C clock frequency in hertz.
func (d *CP2112) SetSmbusClockSpeedHz(clockSpeedHz uint32) error {
	c, err := d.GetSmbusConfiguration()
	if err != nil {
		return err
	}
	c.ClockSpeedHz = clockSpeedHz
	return d.SetSmbusConfiguration(c)
}

func checkDeviceAddress(addr byte) error {
	if (addr & 1) == 1 {
		return fmt.Errorf("invalid device address: 0x%02x. LSB must be zero", addr)
	}
	return nil
}

// TransferDataReadRequest initiates a read operation. deviceAddr is the 7-bit address of the
// device from which data is being read. length is the number of bytes being requested from
// the device.
func (d *CP2112) TransferDataReadRequest(deviceAddr byte, length uint16) error {
	if err := checkDeviceAddress(deviceAddr); err != nil {
		return fmt.Errorf("TransferDataReadRequest: %w", err)
	}
	if length < 1 || length > 512 {
		return fmt.Errorf("TransferDataReadRequest: invalid read length: %d. Must be between 1 and 512", length)
	}
	buf := make([]byte, 4)
	buf[0] = reportIdDataReadRequest
	buf[1] = deviceAddr
	binary.BigEndian.PutUint16(buf[2:4], length)
	if n, err := d.dev.Write(buf); err != nil {
		return fmt.Errorf("TransferDataReadRequest: %w", err)
	} else if n != 4 {
		return fmt.Errorf("TransferDataReadRequest: sent unexpected number of bytes: %d", n)
	}
	return nil
}

func checkReadLength(length uint16) error {
	if length < 1 || length > 512 {
		return fmt.Errorf("invalid read length: %d. Must be between 1 and 512", length)
	}
	return nil
}

// TransferDataWriteReadRequest initiates a address write operation followed by a read operation.
// deviceAddr is the 7-bit address of the device from which data is being read. length is the
// number of bytes being requested from the device. targetAddr is the address of the memory
// location being read on the device. The number of bytes in the targetAddr can be maximum
// 16 bytes.
func (d *CP2112) TransferDataWriteReadRequest(deviceAddr byte, length uint16, targetAddr []byte) error {
	if err := checkDeviceAddress(deviceAddr); err != nil {
		return fmt.Errorf("TransferDataWriteReadRequest: %w", err)
	}
	if err := checkReadLength(length); err != nil {
		return fmt.Errorf("TransferDataWriteReadRequest: %w", err)
	}
	targetAddrLen := len(targetAddr)
	if len(targetAddr) < 1 || len(targetAddr) > 16 {
		return fmt.Errorf("TransferDataWriteReadRequest: invalid targetAddress length: %d", len(targetAddr))
	}
	buf := make([]byte, 5+targetAddrLen)
	buf[0] = reportIdDataWriteReadRequest
	buf[1] = deviceAddr
	binary.BigEndian.PutUint16(buf[2:4], length)
	buf[4] = byte(targetAddrLen)
	copy(buf[5:], targetAddr)
	if n, err := d.dev.Write(buf); err != nil {
		return fmt.Errorf("TransferDataWriteReadRequest: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("TransferDataWriteReadRequest: sent unexpected number of bytes: %d", n)
	}
	return nil
}

// TransferDataReadForceSend forces the device to send a Data Read Response report
// when the next Interrupt IN token arrives. This is essentially polled mode reading.
// The PC should poll using Transfer Status Request first to determine whether data is
// ready. The number of bytes requested can be 1 to 512. If the number of bytes requested
// is greater than the number of valid bytes in the CP2112’s received bytes buffer, only
// the valid bytes will be returned.
//
// This command should only be used when Auto Send Read is set to false. This command
// is ignored when Auto Send Read is set to true. If a transfer is not in progress or
// if no data is in the buffer, this command performs no action. This command can be
// used while a read is in progress to retrieve the data received so far.
func (d *CP2112) TransferDataReadForceSend(length uint16) error {
	if err := checkReadLength(length); err != nil {
		return fmt.Errorf("TransferDataReadForceSend: %w", err)
	}
	buf := make([]byte, 3)
	buf[0] = reportIdDataReadForceSend
	binary.BigEndian.PutUint16(buf[1:3], length)
	if n, err := d.dev.Write(buf); err != nil {
		return fmt.Errorf("TransferDataReadForceSend: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("TransferDataReadForceSend: sent unexpected number of bytes: %d", n)
	}
	return nil
}

// TransferStatus0 is the main status response from a transfer
type TransferStatus0 byte

const (
	Idle          TransferStatus0 = 0x00
	Busy          TransferStatus0 = 0x01
	Complete      TransferStatus0 = 0x02
	CompleteError TransferStatus0 = 0x03
)

// TransferDataReadResponse returns status and data for Data Read Request,
// Data Write Request, and Data Read Force Send.
func (d *CP2112) TransferDataReadResponse() (TransferStatus0, []byte, error) {
	buf := make([]byte, 64)
	if n, err := d.dev.Read(buf); err != nil {
		return 0, nil, fmt.Errorf("TransferDataReadResponse: %w", err)
	} else if n != len(buf) {
		return 0, nil, fmt.Errorf("TransferDataReadResponse: received unexpected number of bytes: %d", n)
	}
	if buf[0] != reportIdDataReadResponse {
		return 0, nil, fmt.Errorf("TransferDataReadResponse: unexpected report ID: %02x", buf[0])
	}
	status := TransferStatus0(buf[1])
	length := buf[2]
	if length > 61 {
		return 0, nil, fmt.Errorf("TransferDataReadResponse: invalid data length: %d", length)
	}
	data := buf[3 : 3+length]
	return status, data, nil
}

// TransferDataWrite initiates a write operation. deviceAddr is the 7-bit address
// of the device to which data is being sent. The address must be between 0xFE and
// 0x02 (the least significant bit is the read/write bit and must be 0). data is
// the actual data being sent over the SMBus to the device. The host can transmit
// 1 to 61 bytes to the CP2112.
func (d *CP2112) TransferDataWrite(deviceAddr byte, data []byte) error {
	if err := checkDeviceAddress(deviceAddr); err != nil {
		return fmt.Errorf("TransferDataWrite: %w", err)
	}
	dataLen := len(data)
	if dataLen < 1 || dataLen > 61 {
		return fmt.Errorf("TransferDataWrite: invalid data length: %d. Must be 1 to 61 bytes", dataLen)
	}
	buf := make([]byte, 3+dataLen)
	buf[0] = reportIdDataWrite
	buf[1] = deviceAddr
	buf[2] = byte(dataLen)
	copy(buf[3:], data)
	if n, err := d.dev.Write(buf); err != nil {
		return fmt.Errorf("TransferDataWrite: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("TransferDataWrite: sent unexpected number of bytes: %d", n)
	}
	return nil
}

// TransferStatusRequest requests a transfer status.
func (d *CP2112) TransferStatusRequest() error {
	buf := []byte{reportIdTransferStatusRequest, 0x01}
	if n, err := d.dev.Write(buf); err != nil {
		return fmt.Errorf("TransferDataReadRequest: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("TransferDataReadRequest: sent unexpected number of bytes: %d", n)
	}
	return nil
}

type TransferStatus1 byte

const (
	AddressAcked    TransferStatus1 = 0x00
	AddressNacked   TransferStatus1 = 0x01
	ReadInProgress  TransferStatus1 = 0x02
	WriteInProgress TransferStatus1 = 0x03

	TimeoutAddressNacked TransferStatus1 = 0x00
	TimeoutBusNotFree    TransferStatus1 = 0x01
	ArbitrationLost      TransferStatus1 = 0x02
	ReadIncomplete       TransferStatus1 = 0x03
	WriteIncomplete      TransferStatus1 = 0x04
	SuccededAfterRetries TransferStatus1 = 0x04
)

type TransferStatus struct {
	Status0 TransferStatus0
	Status1 TransferStatus1
	Status2 uint16
	Status3 uint16
}

// TransferStatusResponse requests a transfer status.
func (d *CP2112) TransferStatusResponse() (TransferStatus, error) {
	buf := make([]byte, 7)
	if n, err := d.dev.Read(buf); err != nil {
		return TransferStatus{}, fmt.Errorf("TransferStatusResponse: %w", err)
	} else if n != len(buf) {
		return TransferStatus{}, fmt.Errorf("TransferStatusResponse: sent unexpected number of bytes: %d", n)
	}
	if buf[0] != reportIdTransferStatusResponse {
		return TransferStatus{}, fmt.Errorf("TransferStatusResponse: unexpected report ID: %02x", buf[0])
	}
	return TransferStatus{
		Status0: TransferStatus0(buf[1]),
		Status1: TransferStatus1(buf[2]),
		Status2: binary.BigEndian.Uint16(buf[3:5]),
		Status3: binary.BigEndian.Uint16(buf[5:7]),
	}, nil
}

// TransferCancel cancels the ongoing transfer
func (d *CP2112) TransferCancel() error {
	buf := []byte{reportIdCancelTransfer, 0x01}
	if n, err := d.dev.Write(buf); err != nil {
		return fmt.Errorf("TransferCancel: %w", err)
	} else if n != len(buf) {
		return fmt.Errorf("TransferCancel: sent unexpected number of bytes: %d", n)
	}
	return nil
}
