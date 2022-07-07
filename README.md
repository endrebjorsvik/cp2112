# CP2112
Golang interface for using the Silicon Labs CP2112 USB to SMBus/I2C bridge through its USB HID interface.

Depends on the [go-hid](https://github.com/sstallion/go-hid) package, which
is based on the [hidapi](https://github.com/signal11/hidapi) C interface.

## Using in PowerShell on Windows

- Download the pre-compiled [hidapi release](https://github.com/libusb/hidapi/tags).
- Unzip to an easily accessible folder like `C:\usr\lib\hidapi`
- Copy the `hidapi\include` folder `hidapi\hidapi`
- Set the two environment variables `CGO_CFLAGS` and `CGO_LDFLAGS` as follows:
  - `$env:CGO_CFLAGS = "-IC:\usr\lib\hidapi-win-0.12.0 -g -O2"`
  - `$env:CGO_LDFLAGS = "-IC:\usr\lib\hidapi-win-0.12.0\x64 -g -O2"`
- Check that the Go environment is properly updated with the `go env` command.
- Copy the `hidapi\x64\hidapi.dll` to your current working directory, or somewhere that is accessible from the `PATH` variable.

To unset an PowerShell environment variable:

    $env:CGO_FLAGS = $null
