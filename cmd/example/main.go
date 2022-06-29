package main

import (
	"flag"
	"fmt"
	"time"

	"github.com/endrebjorsvik/cp2112"
	log "github.com/sirupsen/logrus"
	hid "github.com/sstallion/go-hid"
)

func gpioDemo(dev *cp2112.CP2112) error {

	values, err := dev.GetGpioValues()
	if err != nil {
		return fmt.Errorf("Could not get GPIO values: %w.", err)
	}
	fmt.Println("GPIOs:", values)
	err = dev.SetGpioDirection(2, cp2112.GpioOutput)
	if err != nil {
		return fmt.Errorf("Could not set GPIO direction: %w.", err)
	}
	values, err = dev.GetGpioValues()
	if err != nil {
		return fmt.Errorf("Could not get GPIO values: %w.", err)
	}
	fmt.Println("GPIOs:", values)
	config, err := dev.GetGpioConfiguration()
	if err != nil {
		return fmt.Errorf("Could not get GPIO config: %w.", err)
	}
	fmt.Println("Config:", config)

	all_out := [8]cp2112.GpioDirection{
		cp2112.GpioOutput,
		cp2112.GpioOutput,
		cp2112.GpioOutput,
		cp2112.GpioOutput,
		cp2112.GpioOutput,
		cp2112.GpioOutput,
		cp2112.GpioOutput,
		cp2112.GpioOutput,
	}
	if err := dev.SetGpioDirections(all_out); err != nil {
		return err
	}
	for i := 0; i < 8; i++ {
		off := uint((i - 1) % 8)
		if err := dev.SetGpioValue(off, cp2112.GpioHigh); err != nil {
			return err
		}
		if err := dev.SetGpioValue(uint(i), cp2112.GpioLow); err != nil {
			return err
		}
		time.Sleep(100 * time.Millisecond)
	}
	return nil
}

func smbusDemo(dev *cp2112.CP2112) error {

	if err := dev.EnableRxTxIndicator(true, true); err != nil {
		return fmt.Errorf("could not enable Tx/Rx indicators: %w", err)
	}

	config, err := dev.GetSmbusConfiguration()
	if err != nil {
		return fmt.Errorf("Could not get SMBus config: %w.", err)
	}
	fmt.Println("SMBus Config:", config)
	if config.ClockSpeedHz == 100_000 {
		config.ClockSpeedHz = 400_000
	} else {
		config.ClockSpeedHz = 100_000
	}
	err = dev.SetSmbusConfiguration(config)
	if err != nil {
		return fmt.Errorf("Could not set SMBus config: %w.", err)
	}
	config, err = dev.GetSmbusConfiguration()
	if err != nil {
		return fmt.Errorf("Could not get SMBus config: %w.", err)
	}
	fmt.Println("SMBus Config:", config)
	err = dev.SetSmbusClockSpeedHz(400_000)
	if err != nil {
		return fmt.Errorf("Could not set SMBus clock speed: %w.", err)
	}

	if err := dev.TransferDataWriteReadRequest(12, 2, []byte{2}); err != nil {
		return fmt.Errorf("Could not send WriteRead request: %w.", err)
	}
	if err := dev.TransferDataReadForceSend(2); err != nil {
		return fmt.Errorf("Could not force read: %w.", err)
	}
	st, data, err := dev.TransferDataReadResponse()
	if err != nil {
		return fmt.Errorf("Could not receive TransferDataReadResponse: %w.", err)
	}
	fmt.Printf("SMBus status: %v, length: %v, data: %s\n", st, len(data), data)

	if err := dev.TransferStatusRequest(); err != nil {
		return fmt.Errorf("Could not request transfer status: %w", err)
	}
	status, err := dev.TransferStatusResponse()
	if err != nil {
		return fmt.Errorf("Could not read transfer status: %w", err)
	}
	fmt.Printf("Transfer status: %v\n", status.String())
	return nil
}

func usbDemo(dev *cp2112.CP2112) error {
	lockBits, err := dev.GetLockBits()
	if err != nil {
		return err
	}
	fmt.Printf("Lock bits: %+v\n", lockBits)

	conf, err := dev.GetUSBConfiguration()
	if err != nil {
		return err
	}
	fmt.Printf("USB configuration: %+v\n", conf)
	return nil
}

func main() {
	verbose := flag.Bool("verbose", false, "Increase logging.")
	runGpio := flag.Bool("gpio", false, "Run GPIO demo.")
	runSmbus := flag.Bool("smbus", false, "Run SMBus demo.")
	runUsb := flag.Bool("usb", false, "Run USB demo.")
	deviceIdx := flag.Int("dev", 0, "Device index of compatible CP2112.")
	flag.Parse()

	if *verbose {
		log.SetLevel(log.DebugLevel)
	}

	idx := 0
	fmt.Println("Listing all connected USB HID devices.")
	err := hid.Enumerate(hid.VendorIDAny, hid.ProductIDAny,
		func(info *hid.DeviceInfo) error {
			fmt.Printf(" %v. %s: ID %04x:%04x %s %s, SN: %s, Rel: %v, %v %v, if: %v.\n",
				idx,
				info.Path,
				info.VendorID,
				info.ProductID,
				info.MfrStr,
				info.ProductStr,
				info.SerialNbr,
				info.ReleaseNbr,
				info.Usage,
				info.UsagePage,
				info.InterfaceNbr)
			idx++
			return nil
		})
	if err != nil {
		log.Fatalf("Could not enumerate HID devices: %s", err)
	}

	cpids, err := cp2112.FindCP2112()
	if err != nil {
		log.Fatalf("Could not find CP2112: %s", err)
	}
	if len(cpids) < 1 {
		log.Fatalf("Could not find any CP2112.")
	}
	if *deviceIdx >= len(cpids) {
		log.Fatalf("Invalid device index: %v", *deviceIdx)
	}
	cpid := cpids[*deviceIdx]

	dev, err := cp2112.NewCP2112(cpid.VID, cpid.PID, cpid.Serial)
	if err != nil {
		log.Fatalf("Error opening CP2112: %s", err)
	}
	defer dev.Close()

	version, err := dev.GetVersionInformation()
	if err != nil {
		log.Fatalf("Could not get version: %s.", err)
	}
	fmt.Println("CP2112 version:", version)

	if *runGpio {
		if err := gpioDemo(dev); err != nil {
			log.Fatalf("gpioDemo error: %s", err)
		}
	}

	if *runSmbus {
		if err := smbusDemo(dev); err != nil {
			log.Fatalf("smbusDemo error: %s", err)
		}
	}

	if *runUsb {
		if err := usbDemo(dev); err != nil {
			log.Fatalf("usbDemo error: %s", err)
		}
	}
}
