package waclib

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
)

type wacMessageOutput interface {
	Marshal() []byte
}

type wacMessageInput interface {
	Unmarshal([]byte) error
}

func bufExtract(input []byte, bytes uint32) ([]byte, []byte, error) {
	if uint32(len(input)) < bytes {
		return nil, nil, errors.New("buffer too short")
	}

	return append([]byte{}, input[:bytes]...), input[bytes:], nil
}

func bufExtractLen32(input []byte, maxLen int) ([]byte, []byte, error) {
	if len(input) < 4 {
		return nil, nil, errors.New("could not read length")
	}

	dlen := binary.BigEndian.Uint32(input)
	input = input[4:]
	if dlen > uint32(maxLen) || dlen > uint32(len(input)) {
		return nil, nil, errors.New("payload too long")
	}

	return bufExtract(input, dlen)
}

func bufWriteLen32(buf []byte, data []byte) []byte {
	var l [4]byte
	binary.BigEndian.PutUint32(l[:], uint32(len(data)))

	return append(append(buf, l[:]...), data...)
}

type authSetupResponse struct {
	PublicKey   [32]byte
	Certificate []byte
	Signature   []byte
}

func (auth *authSetupResponse) Unmarshal(data []byte) error {
	var err error

	var public []byte
	if public, data, err = bufExtract(data, 32); err != nil {
		return err
	}
	copy(auth.PublicKey[:], public)

	if auth.Certificate, data, err = bufExtractLen32(data, 1280); err != nil {
		return err
	}

	if auth.Signature, data, err = bufExtractLen32(data, 256); err != nil {
		return err
	}

	if len(data) != 0 {
		return errors.New("extra data received")
	}

	return nil
}

func (auth authSetupResponse) Marshal() []byte {
	buf := make([]byte, len(auth.PublicKey), len(auth.PublicKey)+4+len(auth.Certificate)+4+len(auth.Signature))

	copy(buf, auth.PublicKey[:])
	buf = bufWriteLen32(buf, auth.Certificate)
	buf = bufWriteLen32(buf, auth.Signature)

	return buf
}

type authSetupRequest struct {
	PublicKey [32]byte
}

func (auth *authSetupRequest) Unmarshal(data []byte) error {
	if len(data) == 0 || data[0] != 1 {
		return errors.New("invalid format")
	}

	public, _, err := bufExtract(data[1:], 32)
	copy(auth.PublicKey[:], public)
	return err
}

func (auth authSetupRequest) Marshal() []byte {
	return append([]byte{1}, auth.PublicKey[:]...)
}

type ConfigRequest struct {
	DeviceName string //0x8
	SSID       string //0xC
	Password   string //0xB
	AirplayKey string /* ???, 0x9 */
}

func (config *ConfigRequest) Unmarshal(data []byte) error {
	fields, err := tlvUnmarshal(data)
	if err != nil {
		return err
	}

	for _, m := range fields {
		dString := string(m.Data)
		switch m.Type {
		case 0x8:
			config.DeviceName = dString
		case 0xC:
			config.SSID = dString
		case 0xB:
			config.Password = dString
		case 0x9:
			config.AirplayKey = dString
		}
	}

	return nil
}

func (config ConfigRequest) Marshal() []byte {
	var configMsg []tlvField
	configMsg = append(configMsg, tlvField{
		Type: 0x8,
		Data: []byte(config.DeviceName),
	})
	configMsg = append(configMsg, tlvField{
		Type: 0xC,
		Data: []byte(config.SSID),
	})
	if len(config.Password) >= 8 {
		configMsg = append(configMsg, tlvField{
			Type: 0xB,
			Data: []byte(config.Password),
		})
	}
	if len(config.AirplayKey) > 0 {
		configMsg = append(configMsg, tlvField{
			Type: 0x9,
			Data: []byte(config.AirplayKey),
		})
	}

	return tlvMarshal(configMsg)
}

type ConfigResponse struct {
	IsEmpty bool

	DeviceName      string //0x8
	Model           string //0x7
	SerialNumber    string //0xA
	FirmwareVersion string //0x2
	HardwareVersion string //0x3
	BundleSeed      string //0x1
}

func (config *ConfigResponse) Unmarshal(data []byte) error {
	config.IsEmpty = len(data) == 0

	if config.IsEmpty {
		return nil
	}

	fields, err := tlvUnmarshal(data)
	if err != nil {
		return err
	}

	for _, m := range fields {
		dString := string(m.Data)
		switch m.Type {
		case 0x8:
			config.DeviceName = dString
		case 0x7:
			config.Model = dString
		case 0xA:
			config.SerialNumber = dString
		case 0x2:
			config.FirmwareVersion = dString
		case 0x3:
			config.HardwareVersion = dString
		case 0x1:
			config.BundleSeed = dString
		}
	}

	return nil
}

func (config ConfigResponse) Marshal() []byte {
	if config.IsEmpty {
		return nil
	}

	var configMsg []tlvField
	appendIfGiven := func(Type byte, value string) {
		if value == "" {
			return
		}
		configMsg = append(configMsg, tlvField{
			Type: Type,
			Data: []byte(value),
		})
	}

	appendIfGiven(0x8, config.DeviceName)
	appendIfGiven(0x7, config.Model)
	appendIfGiven(0xA, config.SerialNumber)
	appendIfGiven(0x2, config.FirmwareVersion)
	appendIfGiven(0x3, config.HardwareVersion)
	appendIfGiven(0x1, config.BundleSeed)

	return tlvMarshal(configMsg)
}

type WACBeacon struct {
	MACAddress [6]byte
	Flags      []byte
	Name       string
	Supplier   string
	Type       string
}

func beaconFindRecord(data []byte) ([]byte, error) {
	ieParts, err := tlvUnmarshal(data)
	if err != nil {
		return nil, fmt.Errorf("failed to parse beacon: %s", err)
	}

	for _, m := range ieParts {
		if m.Type == 0xDD && len(m.Data) >= 4 {
			if bytes.Equal(m.Data[:4], []byte{0x00, 0xA0, 0x40, 0x00}) {
				return m.Data[4:], nil
			}
		}
	}

	return nil, errors.New("record not found")
}

func (ie *WACBeacon) Unmarshal(data []byte) error {
	data, err := beaconFindRecord(data)
	if err != nil {
		return err
	}

	fields, err := tlvUnmarshal(data)
	if err != nil {
		return fmt.Errorf("failed to parse configuration IE: %s", err)
	}

	for _, m := range fields {
		dString := string(m.Data)
		switch m.Type {
		case 0x7:
			if len(m.Data) != 6 {
				return errors.New("MAC address has wrong length")
			}
			copy(ie.MACAddress[:], m.Data)
		case 0x0:
			ie.Flags = m.Data
		case 0x1:
			ie.Name = dString
		case 0x2:
			ie.Supplier = dString
		case 0x3:
			ie.Type = dString
		}
	}

	return nil
}

func (ie *WACBeacon) Marshal() []byte {
	var beacon []tlvField
	beacon = append(beacon, tlvField{
		Type: 0x7,
		Data: ie.MACAddress[:],
	})
	beacon = append(beacon, tlvField{
		Type: 0x0,
		Data: ie.Flags,
	})
	if ie.Name != "" {
		beacon = append(beacon, tlvField{
			Type: 0x1,
			Data: []byte(ie.Name),
		})
	}
	if ie.Supplier != "" {
		beacon = append(beacon, tlvField{
			Type: 0x2,
			Data: []byte(ie.Supplier),
		})
	}
	if ie.Type != "" {
		beacon = append(beacon, tlvField{
			Type: 0x3,
			Data: []byte(ie.Type),
		})
	}

	result := []byte{0xDD, 0xFF, 0x00, 0xA0, 0x40, 0x00}
	tmp := tlvMarshal(beacon)
	result = append(result, tmp...)
	result[1] = byte(len(tmp) + 4)
	return result
}
