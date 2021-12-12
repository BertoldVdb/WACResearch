// Package authchip implements a Golang module to talk to an Apple MFI authentication chip.
// The protocol is documented in this software released under the Apache2.0 license:
// https://github.com/apple/HomeKitADK/blob/master/HAP/HAPMFiHWAuth.c
package authchip

import (
	"crypto/rand"
	"crypto/x509"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/BertoldVdb/WACResearch/mfipki"
	"go.mozilla.org/pkcs7"
)

type LogFunc func(format string, params ...interface{})

type I2CFuncType func(tx []byte, rx []byte) (bool, error)
type PowerSwitchType func(enable bool) error

type Chip struct {
	i2cFunc        I2CFuncType
	powerFunc      PowerSwitchType
	i2cMaxTransfer int

	chipInfo ChipInfo

	certRaw    []byte
	certParsed *x509.Certificate

	workMutex   sync.Mutex
	powerChan   chan (struct{})
	powerNeeded bool
	powerOn     bool

	closed bool

	logFunc LogFunc
}

type chipRegister int

const (
	regDeviceVersion                      chipRegister = 0x00
	regAuthenticationRevision             chipRegister = 0x01
	regAuthenticationProtocolMajorVersion chipRegister = 0x02
	regAuthenticationProtocolMinorVersion chipRegister = 0x03
	regErrorCode                          chipRegister = 0x05

	regAuthenticationControlAndStatus chipRegister = 0x10
	regChallengeResponseDataLength    chipRegister = 0x11
	regChallengeResponseData          chipRegister = 0x12

	regChallengeDataLength chipRegister = 0x20
	regChallengeData       chipRegister = 0x21

	regAccessoryCertificateDataLength chipRegister = 0x30
	regAccessoryCertificateData       chipRegister = 0x31

	regSelfTestStatus     chipRegister = 0x40
	regSystemEventCounter chipRegister = 0x4D
)

func (c *Chip) log(format string, params ...interface{}) {
	if c.logFunc != nil {
		c.logFunc(" * "+format, params...)
	}
}

func New(i2cFunc I2CFuncType, powerFunc PowerSwitchType, i2cMaxTransfer int, logFunc LogFunc) (*Chip, error) {
	if i2cMaxTransfer == 0 {
		i2cMaxTransfer = 256
	}

	m := &Chip{
		i2cFunc:        i2cFunc,
		i2cMaxTransfer: i2cMaxTransfer,

		powerChan: make(chan (struct{}), 1),

		logFunc: logFunc,
	}

	m.powerFunc = func(enable bool) error {
		if powerFunc == nil {
			return nil
		}

		if enable {
			m.log("Enabling chip power")
		} else {
			m.log("Disabling chip power")
		}

		return powerFunc(enable)
	}

	var err error

	if err = m.powerFunc(false); err != nil {
		return nil, err
	}

	time.Sleep(100 * time.Millisecond)

	if err = m.powerSet(true); err != nil {
		return nil, err
	}

	if err = m.readChipInfo(); err != nil {
		goto failed
	}

	if err = m.doSelftestChip(); err != nil {
		goto failed
	}

	if err = m.readCertificate(); err != nil {
		goto failed
	}

	if err = m.parseCertificate(); err != nil {
		goto failed
	}

	if err = m.doSelftestSign(); err != nil {
		goto failed
	}

	go m.powerOffWorker()

	return m, nil

failed:
	m.Close()
	return nil, err
}

func (m *Chip) i2cTxfrRetry(tx []byte, rx []byte) error {
	timeout := time.Now().Add(time.Second)

	for time.Now().Before(timeout) {
		ack, err := m.i2cFunc(tx, rx)

		if err != nil {
			return err
		}

		if ack {
			return nil
		}
	}

	return errors.New("no ACK received")
}

func (m *Chip) powerSet(enable bool) error {
	var err error

	if m.closed {
		return errors.New("device is closed")
	}

	if enable && !m.powerOn {
		err = m.powerFunc(true)
		if err == nil {
			m.powerNeeded = true
			m.powerOn = true
		}
		time.Sleep(20 * time.Millisecond)
	} else if !enable {
		m.powerNeeded = false
	}
	select {
	case m.powerChan <- struct{}{}:
	default:
	}

	return err
}

func (m *Chip) powerOffSafe() {
	var buf [1]byte
	var err error
	if m.chipInfo.ProtocolMajorVersion == 2 {
		err = m.regGet(regSystemEventCounter, buf[:])
	}

	if err != nil || buf[0] == 0 {
		m.powerFunc(false)
		m.powerOn = false
	}
}

func (m *Chip) powerOffWorker() {
	for {
		m.workMutex.Lock()
		powerNeeded := m.powerNeeded
		if !powerNeeded && m.powerOn {
			m.powerOffSafe()

			m.workMutex.Unlock()
			time.Sleep(100 * time.Millisecond)

		} else {
			m.workMutex.Unlock()

			_, ok := <-m.powerChan
			if !ok {
				return
			}
		}
	}
}

func (m *Chip) Close() error {
	m.workMutex.Lock()
	defer m.workMutex.Unlock()

	if !m.closed {
		close(m.powerChan)
		m.closed = true
	}

	for m.powerOn {
		time.Sleep(100 * time.Millisecond)
		m.powerOffSafe()
	}

	return nil
}

func (m *Chip) regGet(addr chipRegister, buf []byte) error {
	var tx [1]byte
	tx[0] = byte(addr)

	if err := m.i2cTxfrRetry(tx[:], nil); err != nil {
		return err
	}

	bytesRead := 0
	for bytesRead < len(buf) {
		rdBuf := buf[bytesRead:]
		if len(rdBuf) > m.i2cMaxTransfer {
			rdBuf = rdBuf[:m.i2cMaxTransfer]
		}

		if err := m.i2cTxfrRetry(nil, rdBuf); err != nil {
			return err
		}

		bytesRead += len(rdBuf)
	}

	m.log("Read    0x%02x: %s", addr, hex.EncodeToString(buf))

	return nil
}

func (m *Chip) regSet(addr chipRegister, buf []byte) error {
	tx := make([]byte, len(buf)+1)
	tx[0] = byte(addr)
	copy(tx[1:], buf)

	if len(tx) > m.i2cMaxTransfer {
		return errors.New("i2cMaxTransfer is too small")
	}

	m.log("Writing 0x%02x: %s", addr, hex.EncodeToString(buf))

	return m.i2cTxfrRetry(tx[:], nil)
}

func (m *Chip) errorGetAndClear() error {
	var buf [1]byte
	if err := m.regGet(regErrorCode, buf[:]); err != nil {
		return err
	}

	if buf[0] != 0 {
		return fmt.Errorf("chip error: %02x", buf[0])
	}
	return nil
}

type ChipInfo struct {
	DeviceVersion        byte
	FirmwareVersion      byte
	ProtocolMajorVersion byte
	ProtocolMinorVersion byte

	SignatureLenInput  uint16
	SignatureLenOutput uint16
}

func (m *Chip) readChipInfo() error {
	var chipInfo [4]byte

	m.errorGetAndClear()

	/* Read chip information */
	for i := range chipInfo {
		if err := m.regGet(regDeviceVersion+chipRegister(i), chipInfo[i:i+1]); err != nil {
			return err
		}
	}

	m.chipInfo.DeviceVersion = chipInfo[regDeviceVersion]
	m.chipInfo.FirmwareVersion = chipInfo[regAuthenticationRevision]
	m.chipInfo.ProtocolMajorVersion = chipInfo[regAuthenticationProtocolMajorVersion]
	m.chipInfo.ProtocolMinorVersion = chipInfo[regAuthenticationProtocolMinorVersion]

	if m.chipInfo.ProtocolMajorVersion == 2 {
		m.chipInfo.SignatureLenInput = 20
		m.chipInfo.SignatureLenOutput = 128
	} else if m.chipInfo.ProtocolMajorVersion == 3 {
		m.chipInfo.SignatureLenInput = 32
		m.chipInfo.SignatureLenOutput = 64
	} else {
		return errors.New("unsupported protocol version")
	}

	return m.errorGetAndClear()
}

func (i ChipInfo) String() string {
	devType := "Unknown"
	if i.DeviceVersion == 5 {
		devType = "2.0C"
	} else if i.DeviceVersion == 7 {
		devType = "3.0"
	}
	return fmt.Sprintf("Type=%s FirmwareVersion=%d ProtocolVersion=%d.%d Signature=%d->%d", devType, i.FirmwareVersion, i.ProtocolMajorVersion, i.ProtocolMinorVersion, i.SignatureLenInput, i.SignatureLenOutput)
}

func (m *Chip) ChipInfo() ChipInfo {
	return m.chipInfo
}

func (m *Chip) doSelftestChip() error {
	if m.chipInfo.ProtocolMajorVersion == 2 {
		/* Start self test */
		if err := m.regSet(regSelfTestStatus, []byte{1}); err != nil {
			return err
		}
	}

	var buf [1]byte
	if err := m.regGet(regSelfTestStatus, buf[:]); err != nil {
		return err
	}

	if buf[0]>>6 != 3 {
		return errors.New("self test failed")
	}

	return nil
}

func (m *Chip) readCertificate() error {
	m.errorGetAndClear()

	var buf [2]byte
	if err := m.regGet(regAccessoryCertificateDataLength, buf[:]); err != nil {
		return err
	}

	certLen := int(binary.BigEndian.Uint16(buf[:]))

	if m.chipInfo.ProtocolMajorVersion == 2 {
		if certLen >= 1280 {
			return errors.New("certificate length invalid")
		}
	} else if m.chipInfo.ProtocolMajorVersion == 3 {
		if certLen < 607 || certLen > 609 {
			return errors.New("certificate length invalid")
		}
	}

	certBlob := make([]byte, certLen)
	certIndex := regAccessoryCertificateData
	certOffset := 0

	for certOffset < certLen {
		rest := certBlob[certOffset:]
		if len(rest) > 128 {
			rest = rest[:128]
		}

		if err := m.regGet(certIndex, rest); err != nil {
			return err
		}
		certOffset += len(rest)
		certIndex++
	}

	m.certRaw = certBlob

	return m.errorGetAndClear()
}

func (m *Chip) parseCertificate() error {
	bundle, err := pkcs7.Parse(m.CertificateRawPKCS7())
	if err != nil {
		return err
	}

	if len(bundle.Certificates) != 1 {
		return fmt.Errorf("wrong number of certificates found: %d", len(bundle.Certificates))
	}

	m.certParsed = bundle.Certificates[0]

	return mfipki.CertificateValidate(m.certParsed)
}

func (m *Chip) SignRaw(input []byte) ([]byte, error) {
	if len(input) != int(m.chipInfo.SignatureLenInput) {
		return nil, fmt.Errorf("digest is %d bytes instead of %d bytes", len(input), m.chipInfo.SignatureLenInput)
	}

	m.workMutex.Lock()
	defer m.workMutex.Unlock()

	if err := m.powerSet(true); err != nil {
		return nil, err
	}
	defer m.powerSet(false)

	var buf [2]byte

	m.errorGetAndClear()

	if m.chipInfo.ProtocolMajorVersion == 2 {
		binary.BigEndian.PutUint16(buf[:], uint16(len(input)))
		if err := m.regSet(regChallengeDataLength, buf[:]); err != nil {
			return nil, err
		}

		binary.BigEndian.PutUint16(buf[:], 128)
		if err := m.regSet(regChallengeResponseDataLength, buf[:]); err != nil {
			return nil, err
		}
	}

	if err := m.regSet(regChallengeData, input); err != nil {
		return nil, err
	}

	if err := m.regSet(regAuthenticationControlAndStatus, []byte{1}); err != nil {
		return nil, err
	}

	if err := m.regGet(regAuthenticationControlAndStatus, buf[:1]); err != nil {
		return nil, err
	} else if buf[0] != 0x10 {
		return nil, fmt.Errorf("signing failed: %02x", buf[0])
	}

	if err := m.regGet(regChallengeResponseDataLength, buf[:]); err != nil {
		return nil, err
	}

	response := make([]byte, binary.BigEndian.Uint16(buf[:]))
	if err := m.regGet(regChallengeResponseData, response); err != nil {
		return nil, err
	}

	return response, m.errorGetAndClear()
}

func (m *Chip) SerialNumber() string {
	return m.certParsed.Subject.CommonName
}

func (m *Chip) CertificateRawPKCS7() []byte {
	return m.certRaw
}

func (m *Chip) CertificateX509() *x509.Certificate {
	return m.certParsed
}

func (m *Chip) doSelftestSign() error {
	hashed := make([]byte, m.chipInfo.SignatureLenInput)

	if _, err := rand.Read(hashed); err != nil {
		return err
	}

	signature, err := m.SignRaw(hashed)
	if err != nil {
		return err
	}

	return mfipki.SignatureValidate(m.certParsed.PublicKey, hashed, signature)
}
