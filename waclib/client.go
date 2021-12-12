package waclib

import (
	"bytes"
	"crypto/cipher"
	"crypto/rand"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"time"

	"github.com/aead/ecdh"
)

type WACClient struct {
	client http.Client

	Insecure bool
}

func NewClient() *WACClient {
	tr := &http.Transport{
		MaxIdleConns:       1,
		DisableKeepAlives:  true,
		IdleConnTimeout:    10 * time.Second,
		DisableCompression: true,
		Dial: (&net.Dialer{
			Timeout: 5 * time.Second,
		}).Dial,
	}
	return &WACClient{
		client: http.Client{
			Transport: tr,
			Timeout:   10 * time.Second,
		},
	}
}

func (c *WACClient) exchange(destination string, endpoint string, encrypt cipher.Stream, request []byte) ([]byte, error) {
	if encrypt != nil {
		encrypt.XORKeyStream(request, request)
	}

	req, err := http.NewRequest("POST", "http://"+destination+"/"+endpoint, bytes.NewBuffer(request))
	if err != nil {
		return nil, err
	}
	req.Header.Set("Content-Type", "application/octet-stream")

	resp, err := c.client.Do(req)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	if resp.StatusCode != 200 {
		return nil, fmt.Errorf("request error %s", resp.Status)
	}

	data, err := ioutil.ReadAll(io.LimitReader(resp.Body, 8192))
	if err != nil {
		return nil, err
	}

	if encrypt != nil {
		encrypt.XORKeyStream(data, data)
	}

	return data, nil
}

func (c *WACClient) exchangeMsg(destination string, endpoint string, encrypt cipher.Stream, request wacMessageOutput, dest wacMessageInput) error {
	data, err := c.exchange(destination, endpoint, encrypt, request.Marshal())
	if err != nil {
		return err
	}

	return dest.Unmarshal(data)
}

func (c *WACClient) Configure(destination string, config ConfigRequest) (*ConfigResponse, error) {
	c25519 := ecdh.X25519()

	private, public, err := c25519.GenerateKey(rand.Reader)
	if err != nil {
		return nil, err
	}
	publicByte := public.([32]byte)

	var authSetupResp authSetupResponse
	if err := c.exchangeMsg(destination, "auth-setup", nil, authSetupRequest{PublicKey: publicByte}, &authSetupResp); err != nil {
		return nil, err
	}

	if err := c25519.Check(authSetupResp.PublicKey); err != nil {
		return nil, err
	}

	secret := c25519.ComputeSecret(private, authSetupResp.PublicKey)

	aesStream, err := secretToStreamCipher(secret)
	if err != nil {
		return nil, err
	}

	aesStream.XORKeyStream(authSetupResp.Signature, authSetupResp.Signature)

	parsedCert, err := certificateParse(authSetupResp.Certificate)
	if err != nil {
		return nil, err
	}

	if !c.Insecure {
		if err := signatureVerifyClient(parsedCert, publicByte, authSetupResp.PublicKey, authSetupResp.Signature); err != nil {
			return nil, err
		}
	}

	var configResponse ConfigResponse
	if err := c.exchangeMsg(destination, "config", aesStream, config, &configResponse); err != nil {
		return nil, err
	}

	if configResponse.IsEmpty {
		return nil, nil
	}
	return &configResponse, nil
}

func (c *WACClient) IsConfigured(destination string) (bool, error) {
	_, err := c.exchange(destination, "configured", nil, nil)
	return err == nil, err
}

func main() {
	var c WACClient

	if false {
		/*./resolv -service _mfi-config._tcp -domain local
		2021/11/07 00:09:56 &{{Connect _mfi-config._tcp [] local _mfi-config._tcp.local. Connect._mfi-config._tcp.local. _services._dns-sd._udp.local.} Connect.local. 80 [seed=0 features=4 deviceid=2C:9F:FB:80:55:7D srcvers=1.14 flags=2 protovers=1.0] 255 [192.168.10.1] []}*/

		log.Println(c.Configure("192.168.10.1", ConfigRequest{
			DeviceName: "Test",
			SSID:       "WLAN",
			Password:   "gKQjmMlF9qmWF5E3Mt9Lp52",
		}))
	} else {
		//2021/11/07 00:10:54 &{{Connect _mfi-config._tcp [] local _mfi-config._tcp.local. Connect._mfi-config._tcp.local. _services._dns-sd._udp.local.} Connect.local. 80 [seed=1 features=4 deviceid=2C:9F:FB:80:55:7D srcvers=1.14 flags=2 protovers=1.0] 255 [192.168.0.146] []}
		log.Println(c.IsConfigured("192.168.0.146"))
	}
}
