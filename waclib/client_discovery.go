package waclib

import (
	"context"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"time"

	"github.com/grandcat/zeroconf"
)

type ServiceDiscoveryResult struct {
	DeviceID string
	Seed     int
	Addr     string
}

func ClientDiscoverService(filterDeviceId string) (ServiceDiscoveryResult, error) {
	var result ServiceDiscoveryResult

	/* The resolver is not reused as we are likely switching between networks
	   while using this */
	resolver, err := zeroconf.NewResolver(nil)
	if err != nil {
		return result, err
	}

	results := make(chan *zeroconf.ServiceEntry)
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	go func() {
		err = resolver.Browse(ctx, "_mfi-config._tcp", "local", results)
		if err != nil {
			return
		}
		<-ctx.Done()
	}()

	for m := range results {
		var seed, deviceid string
		for _, m := range m.Text {
			kv := strings.SplitN(m, "=", 2)
			if len(kv) == 2 {
				key := strings.ToLower(kv[0])
				value := kv[1]

				switch key {
				case "seed":
					seed = value
				case "deviceid":
					deviceid = value
				}
			}
		}

		if seed == "" || deviceid == "" {
			continue
		}

		if filterDeviceId != "" && deviceid != filterDeviceId {
			continue
		}

		seedInt, err := strconv.Atoi(seed)
		if err != nil {
			continue
		}

		var addr string
		if len(m.AddrIPv4) > 0 {
			addr = m.AddrIPv4[0].String()
		}
		if len(m.AddrIPv6) > 0 {
			addr = "[" + m.AddrIPv6[0].String() + "]"
		}

		addr += fmt.Sprintf(":%d", m.Port)

		result = ServiceDiscoveryResult{
			DeviceID: deviceid,
			Seed:     seedInt,
			Addr:     addr,
		}
		return result, nil
	}

	return result, errors.New("no results")
}
