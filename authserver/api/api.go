package api

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"io/ioutil"
	"net/http"

	"github.com/BertoldVdb/WACResearch/authchip"
)

type API struct {
	mux  *http.ServeMux
	chip *authchip.Chip
}

const (
	ctBinary string = "application/octet-stream"
	ctPkcs7  string = "application/pkcs7-mime"
	ctJSON   string = "application/json"
)

func New(chip *authchip.Chip) (*API, error) {
	mux := &http.ServeMux{}

	s := &API{
		mux:  mux,
		chip: chip,
	}

	var infoResponse struct {
		Serial           string
		Info             authchip.ChipInfo
		CertificatePKCS7 []byte
	}
	infoResponse.Serial = s.chip.SerialNumber()
	infoResponse.Info = s.chip.ChipInfo()
	infoResponse.CertificatePKCS7 = chip.CertificateRawPKCS7()

	infoJson, err := json.MarshalIndent(&infoResponse, "", "  ")
	if err != nil {
		return nil, err
	}

	var versionResponse bytes.Buffer
	if err := binary.Write(&versionResponse, binary.BigEndian, &infoResponse.Info); err != nil {
		return nil, err
	}

	mux.HandleFunc("/info", sendStatic(ctJSON, infoJson))
	mux.HandleFunc("/cert", sendStatic(ctPkcs7, infoResponse.CertificatePKCS7))
	mux.HandleFunc("/version", sendStatic(ctBinary, versionResponse.Bytes()))
	mux.HandleFunc("/sign", s.signHandler)

	return s, nil
}

func sendStatic(contentType string, data []byte) func(w http.ResponseWriter, r *http.Request) {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.Method != "GET" {
			http.Error(w, "Invalid method", http.StatusMethodNotAllowed)
			return
		}

		w.Header().Set("Content-Type", contentType)
		w.Write(data)
	}
}

func (s *API) signHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method != "POST" {
		http.Error(w, "Invalid method", http.StatusMethodNotAllowed)
		return
	}

	input, err := ioutil.ReadAll(r.Body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	output, err := s.chip.SignRaw(input)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	w.Header().Set("Content-Type", ctBinary)
	w.Write(output)
}

func (s *API) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	s.mux.ServeHTTP(w, r)
}
