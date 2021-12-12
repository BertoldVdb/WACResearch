package main

import (
	"encoding/hex"
	"flag"
	"log"
	"net"

	"github.com/BertoldVdb/WACResearch/waclib"
)

func main() {
	decode := flag.String("decode", "", "Specify beacon IE to decode")

	mac := flag.String("mac", "00:11:22:33:44:55", "Specify MAC address to encode")
	flags := flag.String("flags", "7003", "Specify flags in beacon") /* F003 for Airplay, 7003 for 'normal' accesory */
	name := flag.String("name", "Name", "Device name")
	supplier := flag.String("supplier", "waclib", "Device supplier")
	dType := flag.String("type", "Type", "Device type")

	flag.Parse()

	if *decode != "" {
		data, err := hex.DecodeString(*decode)
		if err != nil {
			log.Fatalln("Decode: failed to parse hex string:", err)
		}

		var ie waclib.WACBeacon
		if err := ie.Unmarshal(data); err != nil {
			log.Fatalln("Decode: invalid beacon:", err)
		}

		log.Printf("Decode:")
		log.Printf("  MAC Address:        %s", net.HardwareAddr(ie.MACAddress[:]))
		log.Printf("  Flags:              %s", hex.EncodeToString(ie.Flags))
		if ie.Name != "" {
			log.Printf("  Name:               %s", ie.Name)
		}
		if ie.Supplier != "" {
			log.Printf("  Supplier:           %s", ie.Supplier)
		}
		if ie.Type != "" {
			log.Printf("  Type:               %s", ie.Type)
		}

		return
	}

	if *mac != "" {
		var ie waclib.WACBeacon
		hwAddr, err := net.ParseMAC(*mac)
		if err != nil {
			log.Fatalln("Encode:", err)
		}
		if len(hwAddr) != 6 {
			log.Fatalln("Encode: MAC address has invalid length")
		}
		copy(ie.MACAddress[:], hwAddr)

		ie.Flags, err = hex.DecodeString(*flags)
		if err != nil {
			log.Fatalln("Encode: failed to parse flags", err)
		}

		ie.Name = *name
		ie.Supplier = *supplier
		ie.Type = *dType

		log.Println("Encode result:", hex.EncodeToString(ie.Marshal()))
	}
}
