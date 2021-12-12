package main

import (
	"flag"
	"log"

	"github.com/BertoldVdb/WACResearch/waclib"
)

func main() {
	destination := flag.String("destination", "", "Skip discovery and use this server")
	deviceName := flag.String("name", "waclib", "Specify the name the device should use")
	ssid := flag.String("ssid", "", "Specify the SSID to connect to")
	password := flag.String("password", "", "Specify the password for the SSID")
	airplay := flag.String("airplay", "", "Specify the AirPlay key")
	insecure := flag.Bool("insecure", false, "Do not attempt to validate device")

	flag.Parse()

	if *ssid == "" {
		log.Fatalln("SSID parameter is requied!")
	}

	client := waclib.NewClient()
	client.Insecure = *insecure

	destAddr := *destination
	deviceID := ""

	if destAddr == "" {
		log.Println("Searching for device")

		discovery, err := waclib.ClientDiscoverService("")
		if err != nil {
			log.Fatalln("Failed to discover:", err)
		}
		log.Printf("Found device %s at %s", discovery.DeviceID, discovery.Addr)

		destAddr = discovery.Addr
		deviceID = discovery.DeviceID
	}

	configReq := waclib.ConfigRequest{
		DeviceName: *deviceName,
		SSID:       *ssid,
		Password:   *password,
		AirplayKey: *airplay,
	}

	response, err := client.Configure(destAddr, configReq)
	if err != nil {
		log.Fatalln("Failed to configure device:", err)
	}

	log.Println("Device configured!")

	if response != nil {
		log.Println("Got extended response:")

		printIfGiven := func(text string, value string) {
			if value == "" {
				return
			}

			log.Println(text, value)
		}

		printIfGiven("  DeviceName      :", response.DeviceName)
		printIfGiven("  Model           :", response.Model)
		printIfGiven("  SerialNumber    :", response.SerialNumber)
		printIfGiven("  FirmwareVersion :", response.FirmwareVersion)
		printIfGiven("  HardwareVersion :", response.HardwareVersion)
		printIfGiven("  BundleSeed      :", response.BundleSeed)
	}

	if *destination != "" {
		return
	}

	var discovery waclib.ServiceDiscoveryResult
	log.Println("Please connect this machine to the configured network. Waiting up to 60 seconds.")
	for i := 0; i < 60; i += 10 {
		discovery, err = waclib.ClientDiscoverService(deviceID)
		if err == nil {
			break
		}
		log.Println("Not ready yet...")
	}

	if err != nil {
		log.Fatalln("Failed to discover device after configuration:", err)
	}
	log.Printf("Found device %s at %s", discovery.DeviceID, discovery.Addr)

	_, err = client.IsConfigured(discovery.Addr)

	if err != nil {
		log.Fatalln("Failed to confirm configuration")
	}

	log.Println("*** Device is configured and connected! ***")
}
