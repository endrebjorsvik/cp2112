package cp2112

import (
	"fmt"
	"time"

	log "github.com/sirupsen/logrus"
	hid "github.com/sstallion/go-hid"
)

// HidInterface defines the required interface for any HID device.
type HidInterface interface {
	SendFeatureReport([]byte) (int, error)
	GetFeatureReport([]byte) (int, error)
	Write([]byte) (int, error)
	ReadWithTimeout([]byte, time.Duration) (int, error)
	Close() error
}

// CP2112 is the primary type for interacting with the SiLabs CP2112
// USB-to-I2C/SMBus controller. The controller also contains a 8 GPIO
// pins that can be controlled through the same interface.
type CP2112 struct {
	dev HidInterface // Open device handle
}

// DevID is USB HID identification that is used to connect to the correct
// HID device.
type DevID struct {
	VID    uint16 // Device Vendor ID
	PID    uint16 // Device Product ID
	Serial string // Device serial number
}

// FindCP2112 finds and returns all compatible CP2112 devices that are
// connected to the system.
func FindCP2112() ([]DevID, error) {
	var devs []DevID
	err := hid.Enumerate(0x10c4, 0xea90,
		func(info *hid.DeviceInfo) error {
			devs = append(devs, DevID{
				VID: info.VendorID, PID: info.ProductID, Serial: info.SerialNbr,
			})
			return nil
		})
	return devs, err
}

// NewCP2112 creates and opens a new CP2112 device.
func NewCP2112(vid, pid uint16, serial string) (*CP2112, error) {
	dev, err := hid.Open(vid, pid, serial)
	if err != nil {
		return nil, fmt.Errorf("could not open HID device 0x%04x:0x%04x (SN: %s): %w", pid, vid, serial, err)
	}
	log.WithFields(log.Fields{
		"VID":    vid,
		"PID":    pid,
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
	reportIdUSBConfiguration       byte = 0x21
	reportIdManufacturerString     byte = 0x22
	reportIdProductString          byte = 0x23
	reportIdSerialString           byte = 0x24
)

// ErrSentUnexpectedBytes indicates that the underlaying HID API sends an unexpected number of bytes.
type ErrSentUnexpectedBytes int

func (e ErrSentUnexpectedBytes) Error() string {
	return fmt.Sprintf("sent unexpected number of bytes: %d", e)
}

// ErrRecvUnexpectedBytes indicates that the underlaying HID API receives an unexpected number of bytes.
type ErrRecvUnexpectedBytes int

func (e ErrRecvUnexpectedBytes) Error() string {
	return fmt.Sprintf("received unexpected number of bytes: %d", e)
}

// ErrUnexpectedReportLength indicates that the HID report parser received a report with unexpected length.
type ErrUnexpectedReportLength int

func (e ErrUnexpectedReportLength) Error() string {
	return fmt.Sprintf("unexpected report length: %d", e)
}

// ErrUnexpectedReportLength indicates that the HID report parser received a report with unexpected HID report ID.
type ErrUnexpectedReportID byte

func (e ErrUnexpectedReportID) Error() string {
	return fmt.Sprintf("unexpected report ID: %02x", int(e))
}

// ErrGpioIndexOutOfRange indicates that an invalid GPIO index was accessed.
type ErrGpioIndexOutOfRange uint

func (e ErrGpioIndexOutOfRange) Error() string {
	return fmt.Sprintf("GPIO index %d is out of range", e)
}

// ErrInvalidSmbusClockSpeed indicates that an invalid SMBus/I2C clock speed was selected.
type ErrInvalidSmbusClockSpeed uint32

func (e ErrInvalidSmbusClockSpeed) Error() string {
	return fmt.Sprintf("invalid SMBus/I2C clock speed: %d Hz", e)
}

// ErrInvalidDeviceAddress indicates that an invalid SMBus/I2C device address was specified.
type ErrInvalidDeviceAddress byte

func (e ErrInvalidDeviceAddress) Error() string {
	return fmt.Sprintf("invalid device address: 0x%02x. Must be <= 127", int(e))
}

// ErrInvalidTransferReadRequestLength indicates that an invalid read request data length was specified.
type ErrInvalidTransferReadRequestLength uint16

func (e ErrInvalidTransferReadRequestLength) Error() string {
	return fmt.Sprintf("invalid data read length: %d. Must be between 1 and 512", e)
}

// ErrInvalidTransferWriteRequestLength indicates that an invalid write request data length was specified.
type ErrInvalidTransferWriteRequestLength uint16

func (e ErrInvalidTransferWriteRequestLength) Error() string {
	return fmt.Sprintf("invalid data write length: %d. Must be between 1 and 61", e)
}

// ErrInvalidTargetAddressLength indicates that an invalid targetAddress length was specified.
type ErrInvalidTargetAddressLength int

func (e ErrInvalidTargetAddressLength) Error() string {
	return fmt.Sprintf("invalid targetAddress length: %d", e)
}

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

func errorWrapper(name string) func(error) error {
	return func(err error) error {
		return fmt.Errorf("%s: %w", name, err)
	}
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
	errf := errorWrapper("ResetDevice")
	buf := []byte{reportIdResetDevice, 0x01}
	if n, err := d.dev.SendFeatureReport(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
	}
	log.WithFields(log.Fields{
		"method": "ResetDevice",
	}).Debugf("Device was reset.")
	return nil
}

// GetVersionInformation reads chip version information from the device.
func (d *CP2112) GetVersionInformation() (Version, error) {
	errf := errorWrapper("GetVersionInformation")
	buf := []byte{reportIdGetVersionInformation, 0, 0}
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return Version{}, errf(err)
	} else if n != 3 {
		return Version{}, errf(ErrRecvUnexpectedBytes(n))
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

func boolsToByte(vals [8]bool) byte {
	var res byte
	for i, v := range vals {
		n := 0
		if v {
			n = 1
		}
		res += byte(n << i)
	}
	return res
}
