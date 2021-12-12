package chipopen

import (
	"errors"
	"fmt"
	"strconv"
	"strings"

	"github.com/BertoldVdb/WACResearch/authchip"
	"github.com/BertoldVdb/WACResearch/authchip/chipopen/mcp2221a"
	"periph.io/x/conn/v3"
	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"
)

func OpenChipUSB(serial string, addr uint, logFunc authchip.LogFunc) (*authchip.Chip, error) {
	var dev *mcp2221a.MCP2221A

	findDevice := func(serial string) (*mcp2221a.MCP2221A, error) {
		devices := mcp2221a.AttachedDevices(mcp2221a.VID, 0xE87B)

		for _, m := range devices {
			if m.Serial == serial || serial == "" {
				hid, err := m.Open()
				if err != nil {
					return nil, err
				}

				return mcp2221a.NewFromDev(hid)
			}
		}

		return nil, errors.New("no device found")
	}

	i2cTxfr := func(tx []byte, rx []byte) (bool, error) {
		fixNack := func(err error) error {
			if strings.Contains(err.Error(), "NACK") {
				return nil
			}
			return err
		}

		lenTx := len(tx)
		lenRx := len(rx)

		if lenTx > 0 {
			err := dev.I2C.Write(true, uint8(addr), tx, uint16(lenTx))
			if err != nil {
				return false, fixNack(err)
			}
		}

		if lenRx > 0 {
			rxData, err := dev.I2C.Read(false, uint8(addr), uint16(lenRx))
			if err != nil {
				return false, fixNack(err)
			}

			copy(rx, rxData)
		}

		return true, nil
	}

	powerSet := func(enable bool) error {
		var err error
		if dev == nil {
			dev, err = findDevice(serial)

			if err != nil {
				return err
			}
		}

		if enable {
			return dev.GPIO.Set(0, 1)
		}

		err = dev.GPIO.Set(0, 0)
		dev.Close()
		dev = nil
		return err
	}

	m, err := authchip.New(i2cTxfr, powerSet, 33, logFunc)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize chip via USB: %v", err)
	}

	return m, nil
}

func OpenChipPlatform(busID string, addr uint, powerpin string, logFunc authchip.LogFunc) (*authchip.Chip, error) {
	if _, err := host.Init(); err != nil {
		return nil, fmt.Errorf("could not init host: %v", err)
	}

	bus, err := i2creg.Open(busID)
	if err != nil {
		return nil, fmt.Errorf("could not open bus: %v", err)
	}

	dev := conn.Conn(&i2c.Dev{Bus: bus, Addr: uint16(addr)})

	maxTxSize := 0
	switch l := dev.(type) {
	case conn.Limits:
		maxTxSize = l.MaxTxSize()
	}

	if maxTxSize == 0 {
		maxTxSize = 128
	}

	i2cTxfr := func(tx []byte, rx []byte) (bool, error) {
		err := dev.Tx(tx, rx)

		if err != nil {
			if strings.Contains(err.Error(), "input/output") {
				err = nil
			}
			if strings.Contains(err.Error(), "no such device") {
				err = nil
			}
			return false, err
		}

		return true, nil
	}

	var powerSet authchip.PowerSwitchType

	if powerpin != "" {
		powerGPIO := gpioreg.ByName(powerpin)
		if powerGPIO == nil {
			return nil, errors.New("power gpio not found")
		}

		powerGPIO.Out(gpio.Low)

		powerSet = func(enable bool) error {
			value := gpio.Low
			if enable {
				value = gpio.High
			}

			return powerGPIO.Out(value)
		}
	}

	m, err := authchip.New(i2cTxfr, powerSet, maxTxSize, logFunc)
	if err != nil {
		return nil, fmt.Errorf("failed to initialize chip: %v", err)
	}

	return m, nil
}

func getPart(parts []string, index int, def string) string {
	if index >= len(parts) {
		return def
	}
	return parts[index]
}

func OpenChip(path string, logOut authchip.LogFunc) (*authchip.Chip, error) {
	parts := strings.Split(path, ":")

	if parts[0] == "usb" {
		serial := getPart(parts, 1, "")
		i2cAddr, err := strconv.ParseUint(getPart(parts, 2, "0x11"), 0, 8)
		if err != nil {
			return nil, err
		}
		return OpenChipUSB(serial, uint(i2cAddr), logOut)
	} else if parts[0] == "platform" {
		bus := getPart(parts, 1, "/dev/i2c-2")
		powerPin := getPart(parts, 2, "")
		i2cAddr, err := strconv.ParseUint(getPart(parts, 3, "0x11"), 0, 8)
		if err != nil {
			return nil, err
		}
		return OpenChipPlatform(bus, uint(i2cAddr), powerPin, logOut)
	}

	return nil, errors.New("device type not suppored, use 'usb' or 'platform'")
}
