package waclib

import (
	"errors"
	"fmt"
	"net"
	"time"

	"github.com/grandcat/zeroconf"
)

type WACServerDiscovery struct {
	seed      byte
	name      string
	port      int
	txtRecord []string

	currentAddr string
	server      *zeroconf.Server
}

func NewServerDiscovery(deviceID string, name string, port int) *WACServerDiscovery {
	if name == "" {
		name = "waclib"
	}

	w := &WACServerDiscovery{
		txtRecord: []string{"seed=unset", "deviceid=" + deviceID, "features=0x04", "srcvers=1.14"},
		name:      name,
		port:      port,
	}

	w.IncrementSeed()

	return w
}

func (w *WACServerDiscovery) IncrementSeed() {
	w.seed++
	w.txtRecord[0] = fmt.Sprintf("seed=%d", w.seed)

	if w.server != nil {
		w.server.SetText(w.txtRecord)
	}
}

func (w *WACServerDiscovery) Stop() {
	if w.server == nil {
		return

	}
	w.server.Shutdown()
	w.server = nil
	w.currentAddr = ""
}

func getIfaceAddressV4(iface *net.Interface) (string, error) {
	addrs, err := iface.Addrs()
	if err != nil {
		return "", err
	}

	for _, m := range addrs {
		k, ok := m.(*net.IPNet)
		if ok {
			if k.IP.To4() == nil {
				continue
			}

			return k.IP.String(), nil
		}
	}

	return "", nil
}

func getIfaceAddressV4Timeout(iface *net.Interface, maxWaitIP time.Duration) (string, error) {
	for deadline := time.Now().Add(maxWaitIP); time.Now().Before(deadline); {
		addr, err := getIfaceAddressV4(iface)
		if err != nil {
			return "", err
		}

		if addr != "" {
			return addr, nil
		}

		time.Sleep(250 * time.Millisecond)
	}

	return "", errors.New("timeout waiting for IPv4 address")
}

func (w *WACServerDiscovery) Start(ifaceName string, maxWaitIP time.Duration) error {
	w.Stop()

	iface, err := net.InterfaceByName(ifaceName)
	if err != nil {
		return err
	}

	addr, err := getIfaceAddressV4Timeout(iface, maxWaitIP)
	if err != nil {
		return err
	}

	server, err := zeroconf.RegisterProxy(w.name, "_mfi-config._tcp", "local.", w.port, w.name, []string{addr}, w.txtRecord, []net.Interface{*iface})
	if err != nil {
		return err
	}
	server.TTL(60)

	w.currentAddr = fmt.Sprintf("%s:%d", addr, w.port)
	w.server = server
	return nil
}

func (w *WACServerDiscovery) CurrentAddress() string {
	return w.currentAddr
}
