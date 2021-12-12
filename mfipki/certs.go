package mfipki

import (
	"crypto/x509"
	_ "embed"
	"fmt"
	"sync"
	"time"
)

/* From: https://opensource.apple.com/source/Security/Security-59754.120.12/tests/TrustTests/EvaluationTests/iAPTests_data.h.auto.html */
//go:embed "cert_aaca2.bin"
var embedCertAaca2 []byte

//go:embed "cert_ap1ca.bin"
var embedCertAp1ca []byte

var roots *x509.CertPool
var intermediates *x509.CertPool
var rootsOnce sync.Once

func initCertPool() {
	addCert := func(data []byte) {
		root, err := x509.ParseCertificate(data)
		if err != nil {
			panic(err)
		}
		roots.AddCert(root)
	}

	rootsOnce.Do(func() {
		roots = x509.NewCertPool()
		intermediates = x509.NewCertPool()

		addCert(embedCertAp1ca)
		addCert(embedCertAaca2)
	})
}

func CertificateValidate(cert *x509.Certificate) error {
	initCertPool()

	/* The example certificate contains "1.2.840.113635.100.6.36": iAPAuthCapabilities, a 32 byte long bitfield.
	 * From: https://opensource.apple.com/source/Security/Security-57740.60.18/OSX/sec/Security/SecCertificate.c.auto.html */
	for _, m := range cert.UnhandledCriticalExtensions {
		if m.String() != "1.2.840.113635.100.6.36" {
			return fmt.Errorf("unsupported critical extension: %v", m)
		}
	}

	certCopy := *cert
	certCopy.UnhandledCriticalExtensions = nil

	opts := x509.VerifyOptions{
		Roots:         roots,
		Intermediates: intermediates,

		CurrentTime: cert.NotBefore.Add(time.Second),
	}

	if _, err := certCopy.Verify(opts); err != nil {
		return fmt.Errorf("failed to verify certificate: %s", err.Error())
	}

	return nil
}
