package cp2112

import (
	"encoding/binary"
	"fmt"
	"time"

	log "github.com/sirupsen/logrus"
)

const (
	smbusClockSpeedHzMin uint32 = 10_000
	smbusClockSpeedHzMax uint32 = 400_000
)

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
	log.WithFields(log.Fields{
		"method": "SmbusConfiguration.toReport",
		"config": c,
	}).Debugf("Creating SMBus configuration buffer.")
	if (c.ClockSpeedHz < smbusClockSpeedHzMin) || (c.ClockSpeedHz > smbusClockSpeedHzMax) {
		return nil, ErrInvalidSmbusClockSpeed(c.ClockSpeedHz)
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
	errf := errorWrapper("SMBus configuration")
	if len(buf) != 14 {
		return SmbusConfiguration{}, errf(ErrUnexpectedReportLength(len(buf)))
	}
	if buf[0] != reportIdSmbusConfiguration {
		return SmbusConfiguration{}, errf(ErrUnexpectedReportID(buf[0]))
	}
	conf := SmbusConfiguration{
		ClockSpeedHz:  binary.BigEndian.Uint32(buf[1:5]),
		DeviceAddress: buf[5],
		AutoSendRead:  buf[6] == 1,
		WriteTimeout:  time.Duration(binary.BigEndian.Uint16(buf[7:9])) * time.Millisecond,
		ReadTimeout:   time.Duration(binary.BigEndian.Uint16(buf[9:11])) * time.Millisecond,
		SclLowTimeout: buf[11] == 1,
		RetryTimes:    binary.BigEndian.Uint16(buf[12:14]),
	}
	log.WithFields(log.Fields{
		"method": "smbusConfigurationFromReport",
		"config": conf,
	}).Debugf("Parsed SMBus configuration.")
	return conf, nil
}

// GetSmbusConfiguration reads the SMBus/I2C configuration from CP2112.
func (d *CP2112) GetSmbusConfiguration() (SmbusConfiguration, error) {
	errf := errorWrapper("GetSmbusConfiguration")
	buf := make([]byte, 14)
	buf[0] = reportIdSmbusConfiguration
	if n, err := d.dev.GetFeatureReport(buf); err != nil {
		return SmbusConfiguration{}, errf(err)
	} else if n != len(buf) {
		return SmbusConfiguration{}, errf(ErrRecvUnexpectedBytes(n))
	}
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("GetSmbusConfiguration received raw SMBus configuration buffer.")
	return smbusConfigurationFromReport(buf)
}

// SetSmbusConfiguration writes the given SMBus/I2C configuration to CP2112.
func (d *CP2112) SetSmbusConfiguration(c SmbusConfiguration) error {
	errf := errorWrapper("SetSmbusConfiguration")
	buf, err := c.toReport()
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("SetSmbusConfiguration sending raw SMBus configuration buffer.")
	if err != nil {
		return errf(err)
	}
	if n, err := d.dev.SendFeatureReport(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
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
	if addr > 127 {
		return ErrInvalidDeviceAddress(addr)
	}
	return nil
}

// TransferDataReadRequest initiates a read operation. deviceAddr is the 7-bit address of the
// device from which data is being read. length is the number of bytes being requested from
// the device.
func (d *CP2112) TransferDataReadRequest(deviceAddr byte, length uint16) error {
	log.WithFields(log.Fields{
		"method":     "TransferDataReadRequest",
		"deviceAddr": deviceAddr,
		"length":     length,
	}).Debugf("Sending Data Read Request.")
	errf := errorWrapper("TransferDataReadRequest")
	if err := checkDeviceAddress(deviceAddr); err != nil {
		return errf(err)
	}
	if err := checkReadLength(length); err != nil {
		return errf(err)
	}
	buf := make([]byte, 4)
	buf[0] = reportIdDataReadRequest
	buf[1] = deviceAddr << 1
	binary.BigEndian.PutUint16(buf[2:4], length)
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferDataReadRequest sending raw buffer.")
	if n, err := d.dev.Write(buf); err != nil {
		return errf(err)
	} else if n != 4 {
		return errf(ErrSentUnexpectedBytes(n))
	}
	return nil
}

func checkReadLength(length uint16) error {
	if length < 1 || length > 512 {
		return ErrInvalidTransferReadRequestLength(length)
	}
	return nil
}

// TransferDataWriteReadRequest initiates a address write operation followed by a read operation.
// deviceAddr is the 7-bit address of the device from which data is being read. length is the
// number of bytes being requested from the device. targetAddr is the address of the memory
// location being read on the device. The number of bytes in the targetAddr can be maximum
// 16 bytes.
func (d *CP2112) TransferDataWriteReadRequest(deviceAddr byte, length uint16, targetAddr []byte) error {
	log.WithFields(log.Fields{
		"method":     "TransferDataWriteReadRequest",
		"deviceAddr": deviceAddr,
		"length":     length,
		"targetAddr": targetAddr,
	}).Debugf("Sending Data Write Read Request.")
	errf := errorWrapper("TransferDataWriteReadRequest")
	if err := checkDeviceAddress(deviceAddr); err != nil {
		return errf(err)
	}
	if err := checkReadLength(length); err != nil {
		return errf(err)
	}
	targetAddrLen := len(targetAddr)
	if len(targetAddr) < 1 || len(targetAddr) > 16 {
		return errf(ErrInvalidTargetAddressLength(len(targetAddr)))
	}
	buf := make([]byte, 5+targetAddrLen)
	buf[0] = reportIdDataWriteReadRequest
	buf[1] = deviceAddr << 1
	binary.BigEndian.PutUint16(buf[2:4], length)
	buf[4] = byte(targetAddrLen)
	copy(buf[5:], targetAddr)
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferDataWriteReadRequest sending raw buffer.")
	if n, err := d.dev.Write(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
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
	log.WithFields(log.Fields{
		"method": "TransferDataReadForceSend",
		"length": length,
	}).Debugf("Sending Data Read Force Send Request.")
	errf := errorWrapper("TransferDataReadForceSend")
	if err := checkReadLength(length); err != nil {
		return errf(err)
	}
	buf := make([]byte, 3)
	buf[0] = reportIdDataReadForceSend
	binary.BigEndian.PutUint16(buf[1:3], length)
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferDataReadForceSend sending raw buffer.")
	if n, err := d.dev.Write(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
	}
	return nil
}

// TransferDataReadResponse returns status and data for Data Read Request,
// Data Write Request, and Data Read Force Send.
func (d *CP2112) TransferDataReadResponse() (TransferStatus0, []byte, error) {
	errf := errorWrapper("TransferDataReadResponse")
	buf := make([]byte, 64)
	if n, err := d.dev.Read(buf); err != nil {
		return 0, nil, errf(err)
	} else if n != len(buf) {
		return 0, nil, errf(ErrRecvUnexpectedBytes(n))
	}
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferDataReadResponse received raw buffer.")
	if buf[0] != reportIdDataReadResponse {
		return 0, nil, errf(ErrUnexpectedReportID(buf[0]))
	}
	status := TransferStatus0(buf[1])
	length := buf[2]
	if length > 61 {
		return 0, nil, fmt.Errorf("TransferDataReadResponse: report contains invalid data length: %v", length)
	}
	data := buf[3 : 3+length]
	log.WithFields(log.Fields{
		"method": "TransferDataReadResponse",
		"status": status,
		"length": length,
		"data":   data,
	}).Debugf("Received Data Read Response.")
	return status, data, nil
}

// TransferDataWrite initiates a write operation. deviceAddr is the 7-bit address
// of the device to which data is being sent and must be <= 127. Read/write bit will
// be padded behind the device address. data is the actual data being sent over the
// SMBus to the device. The host can transmit 1 to 61 bytes to the CP2112.
func (d *CP2112) TransferDataWrite(deviceAddr byte, data []byte) error {
	log.WithFields(log.Fields{
		"method":     "TransferDataWrite",
		"deviceAddr": deviceAddr,
		"data":       data,
	}).Debugf("Sending Data Write Request.")
	errf := errorWrapper("TransferDataWrite")
	if err := checkDeviceAddress(deviceAddr); err != nil {
		return errf(err)
	}
	dataLen := len(data)
	if dataLen < 1 || dataLen > 61 {
		return errf(ErrInvalidTransferWriteRequestLength(dataLen))
	}
	buf := make([]byte, 3+dataLen)
	buf[0] = reportIdDataWrite
	buf[1] = deviceAddr << 1
	buf[2] = byte(dataLen)
	copy(buf[3:], data)
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferDataWrite sending raw buffer.")
	if n, err := d.dev.Write(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
	}
	return nil
}

// TransferStatusRequest requests a transfer status.
func (d *CP2112) TransferStatusRequest() error {
	log.WithFields(log.Fields{
		"method": "TransferStatusRequest",
	}).Debugf("Sending Status Request.")
	errf := errorWrapper("TransferStatusRequest")
	buf := []byte{reportIdTransferStatusRequest, 0x01}
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferStatusRequest sending raw buffer.")
	if n, err := d.dev.Write(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
	}
	return nil
}

// TransferStatus0 is the main status response from a transfer
type TransferStatus0 byte

const (
	Idle            TransferStatus0 = 0x00 // Idle. No transfer has occured since last readout. No other status bits are valid.
	Busy            TransferStatus0 = 0x01 // Transfer is currently ongoing.
	CompleteSuccess TransferStatus0 = 0x02 // Transfer completed successfully.
	CompleteError   TransferStatus0 = 0x03 // Transfer completed with errors.
)

func (s TransferStatus0) IsIdle() bool {
	return s == Idle
}

func (s TransferStatus0) IsBusy() bool {
	return s == Busy
}

func (s TransferStatus0) IsCompleteSuccess() bool {
	return s == CompleteSuccess
}

func (s TransferStatus0) IsCompleteError() bool {
	return s == CompleteError
}

type TransferBusyStatus byte
type TransferCompleteStatus byte

//go:generate stringer -type=TransferBusyStatus,TransferStatus0,TransferCompleteStatus,LockBit,USBPowerMode -output=cp2112_string.go

const (
	// Useful temporary status conditions, but not actual errors. Belongs to Busy.
	AddressAcked    TransferBusyStatus = 0x00 // Address ACKed. OK.
	AddressNacked   TransferBusyStatus = 0x01 // Address NACKed.
	ReadInProgress  TransferBusyStatus = 0x02 // Data read in progress.
	WriteInProgress TransferBusyStatus = 0x03 // Data write in progress.

	InvalidBusyStatus TransferBusyStatus = 0xff // Only used as return value when no other status is valid.

	// Belongs to CompleteError. All are error conditions.
	TimeoutAddressNacked TransferCompleteStatus = 0x00 // Timeout address NACKed.
	TimeoutBusNotFree    TransferCompleteStatus = 0x01 // Timeout bus not free (SCL Low Timeout).
	ArbitrationLost      TransferCompleteStatus = 0x02 // Arbitration lost.
	ReadIncomplete       TransferCompleteStatus = 0x03 // Read incomplete.
	WriteIncomplete      TransferCompleteStatus = 0x04 // Write incomplete.

	// Belongs to CompleteSuccess.
	SuccededAfterRetries TransferCompleteStatus = 0x05 // Succeeded after NumRetries retries.

	InvalidCompleteStatus TransferCompleteStatus = 0xff // Only used as return value when no other status is valid.
)

type TransferStatus struct {
	status0          TransferStatus0
	busyStatus       TransferBusyStatus // Specific busy condition based on Status0.
	completeStatus   TransferCompleteStatus
	numRetries       uint16 // Number of retries before completing, being canceled, or timing out
	numBytesReceived uint16 // Number of received bytes.
}

func (s *TransferStatus) IsIdle() bool {
	return s.status0.IsIdle()
}

func (s *TransferStatus) IsBusy() (bool, TransferBusyStatus) {
	return s.status0.IsBusy(), s.busyStatus
}

func (s *TransferStatus) IsCompleteError() (bool, TransferCompleteStatus) {
	if !s.status0.IsCompleteError() {
		return false, InvalidCompleteStatus
	}
	return true, s.completeStatus
}

func (s *TransferStatus) IsCompleteSuccess() (bool, *TransferCompleteInfo, error) {
	if !s.status0.IsCompleteSuccess() {
		return false, nil, nil
	}
	if s.completeStatus != SuccededAfterRetries {
		return false, nil, fmt.Errorf("unexpected success status: %v", s.completeStatus)
	}
	return true, &TransferCompleteInfo{
		NumRetries:       s.numRetries,
		NumBytesReceived: s.numBytesReceived,
	}, nil
}

func (s *TransferStatus) String() string {
	if s.IsIdle() {
		return "idle"
	}
	if ok, status := s.IsBusy(); ok {
		return fmt.Sprintf("busy with status %s", status)
	}
	if ok, status := s.IsCompleteError(); ok {
		return fmt.Sprintf("transfer failed with error %s", status)
	}
	if ok, status, err := s.IsCompleteSuccess(); err != nil {
		return err.Error()
	} else if ok {
		return status.String()
	}
	return fmt.Sprintf("%d", s)
}

// TransferCompleteInfo contains information from a successful transfer.
type TransferCompleteInfo struct {
	NumRetries       uint16
	NumBytesReceived uint16
}

func (s *TransferCompleteInfo) String() string {
	return fmt.Sprintf("transfered %v bytes with %v retries", s.NumBytesReceived, s.NumRetries)
}

// TransferStatusResponse receives the status response from the previously
// requested transfer status.
func (d *CP2112) TransferStatusResponse() (TransferStatus, error) {
	errf := errorWrapper("TransferStatusResponse")
	buf := make([]byte, 7)
	if n, err := d.dev.Read(buf); err != nil {
		return TransferStatus{}, errf(err)
	} else if n != len(buf) {
		return TransferStatus{}, errf(ErrSentUnexpectedBytes(n))
	}
	log.WithFields(log.Fields{
		"buffer": buf,
	}).Tracef("TransferStatusResponse received raw status buffer.")
	if buf[0] != reportIdTransferStatusResponse {
		return TransferStatus{}, errf(ErrUnexpectedReportID(buf[0]))
	}
	status := TransferStatus0(buf[1])
	busyStatus := InvalidBusyStatus
	completeStatus := InvalidCompleteStatus
	if status.IsBusy() {
		busyStatus = TransferBusyStatus(buf[2])
	}
	if status.IsCompleteSuccess() || status.IsCompleteError() {
		completeStatus = TransferCompleteStatus(buf[2])
	}
	st := TransferStatus{
		status0:          status,
		busyStatus:       busyStatus,
		completeStatus:   completeStatus,
		numRetries:       binary.BigEndian.Uint16(buf[3:5]),
		numBytesReceived: binary.BigEndian.Uint16(buf[5:7]),
	}
	log.WithFields(log.Fields{
		"method":         "TransferStatusResponse",
		"status0":        status,
		"busyStatus":     busyStatus,
		"completeStatus": completeStatus,
		"status":         st,
	}).Debugf("Received Data Read Response.")
	return st, nil
}

// TransferCancel cancels the ongoing transfer
func (d *CP2112) TransferCancel() error {
	log.WithFields(log.Fields{
		"method": "TransferCancel",
	}).Debugf("Sending Cancel Request.")
	errf := errorWrapper("TransferCancel")
	buf := []byte{reportIdCancelTransfer, 0x01}
	if n, err := d.dev.Write(buf); err != nil {
		return errf(err)
	} else if n != len(buf) {
		return errf(ErrSentUnexpectedBytes(n))
	}
	return nil
}

// TODO: Implement high-level functions for reading and writing bytes to register.
