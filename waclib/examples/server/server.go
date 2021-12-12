package main

import (
	"context"
	"errors"
	"flag"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"time"

	"github.com/BertoldVdb/WACResearch/authchip/chipopen"
	"github.com/BertoldVdb/WACResearch/authserver/authclient"
	"github.com/BertoldVdb/WACResearch/waclib"
	"github.com/BertoldVdb/WACResearch/waclib/httpaddhost"
)

func main() {
	var err error
	var chip waclib.AuthHandler

	deviceID := flag.String("deviceid", "00:11:22:33:44:55", "Specifiy the device ID to use")
	port := flag.Int("port", 5555, "Specify port to use")
	ifaceAP := flag.String("apiface", "", "Access point interface")
	ifaceClient := flag.String("clientiface", "wlp1s0", "Client interface")
	cfgscript := flag.String("cfgscript", "", "Script that will configure network configuration")
	honeypot := flag.Bool("honeypot", false, "Log requests but don't act on them")
	chipPath := flag.String("chip", "", "Use local MFI chip")
	chipURL := flag.String("url", "http://127.0.0.1:8066/0", "Use remote MFI chip")

	flag.Parse()

	if *ifaceAP == "" {
		*ifaceAP = *ifaceClient
	}

	closeChan := make(chan os.Signal, 1)
	signal.Notify(closeChan, os.Interrupt)

	if *chipPath != "" {
		chip, err = chipopen.OpenChip(*chipPath, nil)
		if err != nil {
			log.Fatalln("Failed to open chip:", err)
		}

	} else {
		chip, err = authclient.New(*chipURL)
		if err != nil {
			log.Fatalln("Failed to create client:", err)
		}
	}

	defer chip.Close()
	go func() {
		<-closeChan
		chip.Close()
		os.Exit(1)
	}()

	log.Println("Using MFI chip:", chip.SerialNumber())

	wacdisc := waclib.NewServerDiscovery(*deviceID, "waclib", *port)
	defer wacdisc.Stop()

	var cfgValues waclib.ConfigRequest

	updateChan := make(chan (struct{}), 1)
	doUpdate := func(err error) {
		select {
		case updateChan <- struct{}{}:
		default:
		}
	}

	configCb := func(cfg waclib.ConfigRequest) (waclib.ConfigResponse, error) {
		log.Printf("Received configuration:")
		log.Printf("  Device Name: %s", cfg.DeviceName)
		log.Printf("  SSID:        %s", cfg.SSID)
		if cfg.Password != "" {
			log.Printf("  Password:    %s", cfg.Password)
		}
		if cfg.AirplayKey != "" {
			log.Printf("  AirPlay:     %s", cfg.AirplayKey)
		}

		if *honeypot {
			return waclib.ConfigResponse{}, errors.New("not accepting changes")
		}

		cfgValues = cfg
		doUpdate(nil)

		/* Note: more fields can be filled here */
		return waclib.ConfigResponse{
			DeviceName: cfg.DeviceName,
		}, nil
	}

	doneCb := func() {
		log.Println("Connection confirmed.")
		doUpdate(nil)
	}

	wacserver, err := waclib.NewServer(chip, !*honeypot, configCb, doneCb)
	if err != nil {
		log.Println("Failed to create server:", err)
		return
	}

	discoverAndServe := func(iface string, stopChan chan (struct{})) error {
		httpserver := &http.Server{
			Handler: wacserver,

			ReadTimeout:       30 * time.Second,
			WriteTimeout:      30 * time.Second,
			IdleTimeout:       30 * time.Second,
			ReadHeaderTimeout: 30 * time.Second,
		}

		wacdisc.Stop()
		wacdisc.IncrementSeed()
		if err := wacdisc.Start(iface, 30*time.Second); err != nil {
			return err
		}

		listener, err := net.Listen("tcp", wacdisc.CurrentAddress())
		if err != nil {
			return err
		}

		retChan := make(chan (error))

		go func() {
			retChan <- httpserver.Serve(httpaddhost.Wrap(listener))
		}()

		select {
		case err = <-retChan:
			return err
		case <-stopChan:
		}

		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		httpserver.Shutdown(ctx)
		cancel()

		<-retChan

		return nil
	}

	if err := discoverAndServe(*ifaceAP, updateChan); err != nil {
		log.Println("HTTP server error:", err)
		return
	}

	if *cfgscript != "" {
		if err := exec.Command(*cfgscript, cfgValues.SSID, cfgValues.Password, cfgValues.DeviceName, cfgValues.AirplayKey).Run(); err != nil {
			log.Println("Failed to set network configuration:", err)
			return
		}
	}
	_ = wacserver.AckConfigurationChange(true)

	if err := discoverAndServe(*ifaceClient, updateChan); err != nil {
		log.Println("HTTP server error:", err)
	}
}
