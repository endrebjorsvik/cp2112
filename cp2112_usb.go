package cp2112

import (
	"encoding/binary"
	"strings"

	log "github.com/sirupsen/logrus"
	"golang.org/x/text/encoding/unicode"
)

// LockBit describes the programmability state of a programmable (non-volatile) field.
type LockBit uint8

const (
	Programmed   LockBit = 0
	Unprogrammed LockBit = 1
)

// LockBits containts the programmability state of all the programmable (non-volatile) fields of the device.
type LockBits struct {
	VID                LockBit
	PID                LockBit
	MaxPower           LockBit
	PowerMode          LockBit
	ReleaseVersion     LockBit
	ManufacturerString LockBit
	ProductString      LockBit
	SerialString       LockBit
}

func newLockBits(b byte) LockBits {
	vals := byteToInts(b)
	return LockBits{
		VID:                LockBit(vals[0]),
		PID:                LockBit(vals[1]),
		MaxPower:           LockBit(vals[2]),
		PowerMode:          LockBit(vals[3]),
		ReleaseVersion:     LockBit(vals[4]),
		ManufacturerString: LockBit(vals[5]),
		ProductString:      LockBit(vals[6]),
		SerialString:       LockBit(vals[7]),
	}
}

// GetLockBits reads the current lock byte for the non-volatile
// programmable fields of the CP2112.
func (d *CP2112) GetLockBits() (LockBits, error) {
	errf := errorWrapper("GetLockBits")
	buf := []byte{reportIdLockByte, 0xff}
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return LockBits{}, errf(err)
	} else if n != len(buf) {
		return LockBits{}, errf(ErrRecvUnexpectedBytes(n))
	}
	bits := newLockBits(buf[0])
	log.WithFields(log.Fields{
		"method": "GetLockBits",
		"bits":   bits,
	}).Debugf("Got lock bits of device.")
	return bits, nil
}

type USBConfiguration struct {
	VID            uint16         // VID is the USB Vendor ID.
	PID            uint16         // PID is the USB Product ID.
	MaxPower       int            // Maximum current requested by the device from the USB host in bus-powered mode.
	PowerMode      USBPowerMode   // Indicates whether the device is operating in Bus-powered or Self-powered mode.
	ReleaseVersion ReleaseVersion // ReleaseVersion is a user-programmable value.
	Mask           LockBits       // Programmability state of each programmable field.
}

// USBPowerMode indicates whether the USB device is operating in Bus-powered or Self-powered mode.
type USBPowerMode uint8

const (
	BusPowered              USBPowerMode = 0
	SelfPoweredRegulatorOff USBPowerMode = 1
	SelfPoweredRegulatorOn  USBPowerMode = 2
)

// ReleaseVersion is a user-programmable value in the device.
type ReleaseVersion struct {
	Major uint8
	Minor uint8
}

func usbConfigurationFromReport(buf []byte) (USBConfiguration, error) {
	errf := errorWrapper("usbConfigurationFromReport")
	if len(buf) != 10 {
		return USBConfiguration{}, errf(ErrUnexpectedReportLength(len(buf)))
	}
	if buf[0] != reportIdUSBConfiguration {
		return USBConfiguration{}, errf(ErrUnexpectedReportID(buf[0]))
	}
	return USBConfiguration{
		VID:       binary.LittleEndian.Uint16(buf[1:3]),
		PID:       binary.LittleEndian.Uint16(buf[3:5]),
		MaxPower:  2 * int(buf[5]),
		PowerMode: USBPowerMode(buf[6]),
		ReleaseVersion: ReleaseVersion{
			Major: buf[7],
			Minor: buf[8],
		},
		Mask: newLockBits(buf[9]),
	}, nil
}

// GetUSBConfiguration reads the current UsbConfiguration from the device.
func (d *CP2112) GetUSBConfiguration() (USBConfiguration, error) {
	errf := errorWrapper("GetUSBConfiguration")
	buf := make([]byte, 10)
	buf[0] = reportIdUSBConfiguration
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return USBConfiguration{}, errf(err)
	} else if n != len(buf) {
		return USBConfiguration{}, errf(ErrRecvUnexpectedBytes(n))
	}
	config, err := usbConfigurationFromReport(buf)
	if err != nil {
		return USBConfiguration{}, errf(err)
	}
	log.WithFields(log.Fields{
		"method": "GetUSBConfiguration",
		"config": config,
	}).Debugf("Got USB configuration of device.")
	return config, nil
}

func utf16LEToString(b []byte) (string, error) {
	// USB string descriptors must use UTF-16-LE encoding according to:
	// "Unicode Engineering Change Notice to the USB 2.0 specification as of February 21, 2005"
	// https://www.usb.org/document-library/usb-20-specification
	enc := unicode.UTF16(unicode.LittleEndian, unicode.IgnoreBOM)
	decoder := enc.NewDecoder()
	d, err := decoder.Bytes(b)
	if err != nil {
		return "", err
	}
	return string(d), nil
}

func (d *CP2112) getUSBStringDescriptor(title string, reportID byte) (string, error) {
	errf := errorWrapper(title)
	buf := make([]byte, 63)
	buf[0] = reportID
	buf[1] = 60 // Maximum length
	buf[2] = 0x03
	n, err := d.dev.GetFeatureReport(buf)
	if err != nil {
		return "", errf(err)
	}
	log.WithFields(log.Fields{
		"title":    title,
		"reportID": reportID,
		"buffer":   buf,
	}).Debugf("Got raw USB string descriptor from device.")
	if buf[0] != reportID {
		return "", errf(ErrUnexpectedReportID(buf[0]))
	}
	if n < int(buf[1]) {
		return "", errf(ErrRecvUnexpectedBytes(n))
	}
	s, err := utf16LEToString(buf[3:])
	s = strings.TrimRight(s, "\x00")
	if err != nil {
		return "", errf(err)
	}
	log.WithFields(log.Fields{
		"title":    title,
		"reportID": reportID,
		"result":   s,
	}).Debugf("Got USB string descriptor from device.")
	return s, nil
}

// GetUSBManufacturerString reads the USB Manufacturer string descriptor from the device.
func (d *CP2112) GetUSBManufacturerString() (string, error) {
	return d.getUSBStringDescriptor("GetUSBManufacturerString", reportIdManufacturerString)
}

// GetUSBProductString reads the USB Product string descriptor from the device.
func (d *CP2112) GetUSBProductString() (string, error) {
	return d.getUSBStringDescriptor("GetUSBProductString", reportIdProductString)
}

// GetUSBSerialString reads the USB Serial string descriptor from the device.
func (d *CP2112) GetUSBSerialString() (string, error) {
	return d.getUSBStringDescriptor("GetUSBSerialString", reportIdSerialString)
}
