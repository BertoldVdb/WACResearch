package main

import (
	"context"
	"crypto"
	"crypto/hmac"
	"crypto/subtle"
	"encoding/hex"
	"encoding/json"
	"flag"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"time"

	"github.com/BertoldVdb/WACResearch/authchip/chipopen"
	"github.com/BertoldVdb/WACResearch/authserver/api"
	"github.com/BertoldVdb/go-misc/httplog"
)

func main() {
	apiKey := flag.String("apikey", "", "API key to use")
	address := flag.String("addr", ":8066", "Address to listen on")
	verbose := flag.Bool("verbose", false, "Enable verbose logging")

	flag.Parse()

	if *apiKey != "" {
		user, pass := authCalculate(*apiKey, "example", time.Now().AddDate(10, 0, 0))
		log.Printf("Password for username '%s': %s", user, pass)
	}

	closeChan := make(chan os.Signal, 1)
	signal.Notify(closeChan, os.Interrupt)

	logOut := log.Printf
	if !*verbose {
		logOut = nil
	}

	var mux http.ServeMux
	serials := make([]string, 0, len(flag.Args()))

	for i, m := range flag.Args() {
		log.Printf("Initializing chip '%s':", m)

		chip, err := chipopen.OpenChip(m, logOut)
		if err != nil {
			log.Printf(" -> Failed to open: %v", err)
			continue
		}
		defer chip.Close()

		log.Println(" -> Chip ready:", chip.ChipInfo())

		serial := chip.SerialNumber()

		api, err := api.New(chip)
		if err != nil {
			log.Println(" -> Failed to create API:", err)
			return
		}

		log.Printf(" -> Registering as '%s' and '%d'", serial, i)
		mux.Handle("/"+serial+"/", http.StripPrefix("/"+serial, api))
		mux.Handle("/"+strconv.Itoa(i)+"/", http.StripPrefix("/"+strconv.Itoa(i), api))

		serials = append(serials, serial)
	}

	if len(serials) == 0 {
		log.Println("No devices available")
		return
	}

	serialJson, err := json.MarshalIndent(&serials, "", "  ")
	if err != nil {
		log.Println(err)
		return
	}

	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write(serialJson)
	})

	logger := httplog.HTTPLog{
		LogOut:     log.Printf,
		ServerName: "MFIAuth",
	}

	server := &http.Server{
		Addr:      *address,
		Handler:   logger.GetHandler(authProcess(mux.ServeHTTP, *apiKey)),

		ReadTimeout:       30 * time.Second,
		WriteTimeout:      30 * time.Second,
		IdleTimeout:       30 * time.Second,
		ReadHeaderTimeout: 30 * time.Second,
	}

	go func() {
		log.Printf("Starting server on: http://%s", *address)
		log.Println("Server stopped:", server.ListenAndServe())

		select {
		case closeChan <- nil:
		default:
		}
	}()

	<-closeChan
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	server.Shutdown(ctx)
	cancel()
}

func authCalculate(authKey string, suffix string, expiry time.Time) (string, string) {
	user := strconv.FormatInt(expiry.Unix(), 10)

	if suffix != "" {
		user += "$" + suffix
	}

	h := hmac.New(crypto.SHA256.New, []byte(authKey))
	h.Write([]byte(user))
	return user, hex.EncodeToString(h.Sum(nil))
}

func authProcess(handler http.HandlerFunc, authKey string) http.HandlerFunc {
	if len(authKey) == 0 {
		return handler
	}

	failed := func(rw http.ResponseWriter) {
		rw.Header().Set("WWW-Authenticate", "Basic")
		rw.WriteHeader(http.StatusUnauthorized)
	}

	return func(rw http.ResponseWriter, rq *http.Request) {
		user, pwd, ok := rq.BasicAuth()
		if !ok {
			failed(rw)
			return
		}

		pwdDec, err := hex.DecodeString(pwd)
		if err != nil {
			failed(rw)
			return
		}

		h := hmac.New(crypto.SHA256.New, []byte(authKey))
		h.Write([]byte(user))

		if subtle.ConstantTimeCompare(pwdDec, h.Sum(nil)) != 1 {
			failed(rw)
			return
		}

		parts := strings.SplitN(user, "$", 2)

		expiry, err := strconv.ParseInt(parts[0], 10, 64)
		if err != nil || time.Now().Unix() > expiry {
			failed(rw)
			return
		}

		handler(rw, rq)
	}
}
