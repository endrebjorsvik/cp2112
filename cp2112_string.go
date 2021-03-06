// Code generated by "stringer -type=TransferBusyStatus,TransferStatus0,TransferCompleteStatus,LockBit,USBPowerMode -output=cp2112_string.go"; DO NOT EDIT.

package cp2112

import "strconv"

func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[AddressAcked-0]
	_ = x[AddressNacked-1]
	_ = x[ReadInProgress-2]
	_ = x[WriteInProgress-3]
	_ = x[InvalidBusyStatus-255]
}

const (
	_TransferBusyStatus_name_0 = "AddressAckedAddressNackedReadInProgressWriteInProgress"
	_TransferBusyStatus_name_1 = "InvalidBusyStatus"
)

var (
	_TransferBusyStatus_index_0 = [...]uint8{0, 12, 25, 39, 54}
)

func (i TransferBusyStatus) String() string {
	switch {
	case i <= 3:
		return _TransferBusyStatus_name_0[_TransferBusyStatus_index_0[i]:_TransferBusyStatus_index_0[i+1]]
	case i == 255:
		return _TransferBusyStatus_name_1
	default:
		return "TransferBusyStatus(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[Idle-0]
	_ = x[Busy-1]
	_ = x[CompleteSuccess-2]
	_ = x[CompleteError-3]
}

const _TransferStatus0_name = "IdleBusyCompleteSuccessCompleteError"

var _TransferStatus0_index = [...]uint8{0, 4, 8, 23, 36}

func (i TransferStatus0) String() string {
	if i >= TransferStatus0(len(_TransferStatus0_index)-1) {
		return "TransferStatus0(" + strconv.FormatInt(int64(i), 10) + ")"
	}
	return _TransferStatus0_name[_TransferStatus0_index[i]:_TransferStatus0_index[i+1]]
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[TimeoutAddressNacked-0]
	_ = x[TimeoutBusNotFree-1]
	_ = x[ArbitrationLost-2]
	_ = x[ReadIncomplete-3]
	_ = x[WriteIncomplete-4]
	_ = x[SuccededAfterRetries-5]
	_ = x[InvalidCompleteStatus-255]
}

const (
	_TransferCompleteStatus_name_0 = "TimeoutAddressNackedTimeoutBusNotFreeArbitrationLostReadIncompleteWriteIncompleteSuccededAfterRetries"
	_TransferCompleteStatus_name_1 = "InvalidCompleteStatus"
)

var (
	_TransferCompleteStatus_index_0 = [...]uint8{0, 20, 37, 52, 66, 81, 101}
)

func (i TransferCompleteStatus) String() string {
	switch {
	case i <= 5:
		return _TransferCompleteStatus_name_0[_TransferCompleteStatus_index_0[i]:_TransferCompleteStatus_index_0[i+1]]
	case i == 255:
		return _TransferCompleteStatus_name_1
	default:
		return "TransferCompleteStatus(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[Programmed-0]
	_ = x[Unprogrammed-1]
}

const _LockBit_name = "ProgrammedUnprogrammed"

var _LockBit_index = [...]uint8{0, 10, 22}

func (i LockBit) String() string {
	if i >= LockBit(len(_LockBit_index)-1) {
		return "LockBit(" + strconv.FormatInt(int64(i), 10) + ")"
	}
	return _LockBit_name[_LockBit_index[i]:_LockBit_index[i+1]]
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[BusPowered-0]
	_ = x[SelfPoweredRegulatorOff-1]
	_ = x[SelfPoweredRegulatorOn-2]
}

const _USBPowerMode_name = "BusPoweredSelfPoweredRegulatorOffSelfPoweredRegulatorOn"

var _USBPowerMode_index = [...]uint8{0, 10, 33, 55}

func (i USBPowerMode) String() string {
	if i >= USBPowerMode(len(_USBPowerMode_index)-1) {
		return "USBPowerMode(" + strconv.FormatInt(int64(i), 10) + ")"
	}
	return _USBPowerMode_name[_USBPowerMode_index[i]:_USBPowerMode_index[i+1]]
}
