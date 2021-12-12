package authclient

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"io/ioutil"
	"net/http"
	"time"
)

type AuthClient struct {
	client http.Client
	url    string

	info struct {
		Serial           string
		CertificatePKCS7 []byte
	}
}

func New(url string) (*AuthClient, error) {
	c := &AuthClient{
		client: http.Client{
			Timeout: 10 * time.Second,
		},

		url: url,
	}

	infoRaw, err := c.doReq("info", nil)
	if err != nil {
		return nil, err
	}

	if err := json.Unmarshal(infoRaw, &c.info); err != nil {
		return nil, err
	}

	return c, nil
}

func (c *AuthClient) doReq(endpoint string, body []byte) ([]byte, error) {
	var rdr io.Reader
	t := "GET"

	if body != nil {
		rdr = bytes.NewBuffer(body)
		t = "POST"
	}

	req, err := http.NewRequest(t, c.url+"/"+endpoint, rdr)
	if err != nil {
		return nil, err
	}

	if body != nil {
		req.Header.Set("Content-Type", "application/octet-stream")
	}

	resp, err := c.client.Do(req)
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	if resp.StatusCode != 200 {
		return nil, fmt.Errorf("request error %s", resp.Status)
	}

	return ioutil.ReadAll(io.LimitReader(resp.Body, 8192))
}

func (c *AuthClient) SignRaw(input []byte) ([]byte, error) {
	return c.doReq("sign", input)
}

func (c *AuthClient) CertificateRawPKCS7() []byte {
	return c.info.CertificatePKCS7
}

func (c *AuthClient) SerialNumber() string {
	return c.info.Serial
}

func (c *AuthClient) Close() error {
	return nil
}
