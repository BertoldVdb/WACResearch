package waclib

import (
	"crypto/rand"
	"crypto/x509"
	"errors"
	"net/http"

	"github.com/aead/ecdh"
)

type AuthHandler interface {
	SerialNumber() string
	SignRaw(input []byte) ([]byte, error)
	CertificateRawPKCS7() []byte
	Close() error
}

type WACServer struct {
	configCb ConfigCbFunc
	doneCb   DoneCbFunc

	sessions *serverSessionManager

	auth              AuthHandler
	certificateRaw    []byte
	certificateParsed *x509.Certificate

	mux http.ServeMux
}

type ConfigCbFunc func(cfg ConfigRequest) (ConfigResponse, error)
type DoneCbFunc func()

func NewServer(auth AuthHandler, singleSession bool, configCb ConfigCbFunc, doneCb DoneCbFunc) (*WACServer, error) {
	s := &WACServer{
		configCb: configCb,
		doneCb:   doneCb,

		sessions:       newServerSessionManager(singleSession),
		auth:           auth,
		certificateRaw: auth.CertificateRawPKCS7(),
	}

	var err error
	s.certificateParsed, err = certificateParse(s.certificateRaw)
	if err != nil {
		return nil, err
	}

	s.registerEndpoints()

	return s, nil
}

func (s *WACServer) handleAuthSetup(session *serverSession, inT wacMessageInput) (wacMessageOutput, error) {
	authSetupRequest := inT.(*authSetupRequest)
	var authSetupResp authSetupResponse

	c25519 := ecdh.X25519()

	private, public, err := c25519.GenerateKey(rand.Reader)
	if err != nil {
		return nil, err
	}
	authSetupResp.PublicKey = public.([32]byte)
	authSetupResp.Certificate = s.certificateRaw

	if err := c25519.Check(authSetupRequest.PublicKey); err != nil {
		return nil, err
	}

	aesStream, err := secretToStreamCipher(c25519.ComputeSecret(private, authSetupRequest.PublicKey))
	if err != nil {
		return nil, err
	}
	session.stream = aesStream

	hashed, err := signatureCreateHash(false, s.certificateParsed, authSetupResp.PublicKey, authSetupRequest.PublicKey)
	if err != nil {
		return nil, err
	}

	authSetupResp.Signature, err = s.auth.SignRaw(hashed)
	if err != nil {
		return nil, err
	}

	session.stream.XORKeyStream(authSetupResp.Signature, authSetupResp.Signature)
	session.state = 1

	return authSetupResp, nil
}

func (s *WACServer) handleConfig(session *serverSession, inT wacMessageInput) (wacMessageOutput, error) {
	in := inT.(*ConfigRequest)

	if s.configCb == nil {
		return nil, errors.New("no callback specified")
	}

	result, err := s.configCb(*in)
	if err != nil {
		return nil, err
	}

	session.state = 2
	session.allowRestart = false

	return result, nil
}

func (s *WACServer) handleConfigured(session *serverSession, inT wacMessageInput) (wacMessageOutput, error) {
	session.state = 4

	if s.doneCb != nil {
		s.doneCb()
	}

	return nil, nil
}

func (s *WACServer) AckConfigurationChange(sucess bool) error {
	session := s.sessions.Get("", 2)
	if session == nil {
		return errors.New("session not found")
	}
	session.Lock()
	defer session.Unlock()

	if sucess {
		session.state = 3
	} else {
		session.state = -1
		session.allowRestart = true
	}

	return nil
}

func (s *WACServer) registerEndpoints() {
	s.mux.HandleFunc("/auth-setup", s.makeHandler(0, false, func() wacMessageInput { return &authSetupRequest{} }, s.handleAuthSetup))
	s.mux.HandleFunc("/config", s.makeHandler(1, true, func() wacMessageInput { return &ConfigRequest{} }, s.handleConfig))
	s.mux.HandleFunc("/configured", s.makeHandler(3, true, nil, s.handleConfigured))
}

func (s *WACServer) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	s.mux.ServeHTTP(w, r)
}
