package main

import (
	"flag"
	"fmt"
	"time"

	"github.com/endrebjorsvik/cp2112"
	log "github.com/sirupsen/logrus"
	"github.com/sstallion/go-hid"
)

func main() {
	verbose := flag.Bool("verbose", false, "Increase logging.")
	flag.Parse()

	if *verbose {
		log.SetLevel(log.DebugLevel)
	}

	fmt.Println("Listing all connected USB HID devices.")
	hid.Enumerate(hid.VendorIDAny, hid.ProductIDAny,
		func(info *hid.DeviceInfo) error {
			fmt.Printf("%s: ID %04x:%04x %s %s, SN: %s, Rel: %d, %d %d, if: %d.\n",
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
			return nil
		})

	cpids, err := cp2112.FindCP2112()
	if err != nil {
		log.Fatalf("Could not find CP2112: %s", err)
	}
	if len(cpids) < 1 {
		log.Fatalf("Could not find any CP2112.")
	}
	cpid := cpids[0]

	dev, err := cp2112.NewCP2112(cpid.Vid, cpid.Pid, cpid.Serial)
	if err != nil {
		log.Fatalf("Error opening CP2112: %s", err)
	}
	defer dev.Close()

	version, err := dev.GetVersionInformation()
	if err != nil {
		log.Fatalf("Could not get version: %s.", err)
	}

	fmt.Println("CP2112 version:", version)
	gpio, err := dev.GetGpioValues()
	if err != nil {
		log.Fatalf("Could not get GPIO values: %s.", err)
	}
	fmt.Println("GPIOs:", gpio)
	err = dev.SetGpioDirection(2, cp2112.GpioOutput)
	if err != nil {
		log.Fatalf("Could not set GPIO direction: %s.", err)
	}
	gpio, err = dev.GetGpioValues()
	if err != nil {
		log.Fatalf("Could not get GPIO values: %s.", err)
	}
	fmt.Println("GPIOs:", gpio)
	config, err := dev.GetGpioConfiguration()
	if err != nil {
		log.Fatalf("Could not get GPIO config: %s.", err)
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
	dev.SetGpioDirections(all_out)
	for i := 0; i < 8; i++ {
		off := uint((i - 1) % 8)
		dev.SetGpioValue(off, cp2112.GpioHigh)
		dev.SetGpioValue(uint(i), cp2112.GpioLow)
		time.Sleep(100 * time.Millisecond)
	}

	smbus, err := dev.GetSmbusConfiguration()
	if err != nil {
		log.Fatalf("Could not get SMBus config: %s.", err)
	}
	fmt.Println("SMBus Config:", smbus)
	if smbus.ClockSpeedHz == 100_000 {
		smbus.ClockSpeedHz = 400_000
	} else {
		smbus.ClockSpeedHz = 100_000
	}
	err = dev.SetSmbusConfiguration(smbus)
	if err != nil {
		log.Fatalf("Could not set SMBus config: %s.", err)
	}
	smbus, err = dev.GetSmbusConfiguration()
	if err != nil {
		log.Fatalf("Could not get SMBus config: %s.", err)
	}
}
