package waclib

import (
	"crypto"
	"crypto/aes"
	"crypto/cipher"
	"crypto/ecdsa"
	"crypto/rsa"
	"crypto/sha1"
	"crypto/x509"
	"fmt"

	"github.com/BertoldVdb/WACResearch/mfipki"
	"go.mozilla.org/pkcs7"
)

// Crypto algorithm from: https://openairplay.github.io/airplay-spec/audio/rtsp_requests/post_auth_setup.html
func secretToStreamCipher(secret []byte) (cipher.Stream, error) {
	keySplit := sha1.New()
	keySplit.Write([]byte("AES-KEY"))
	keySplit.Write(secret)
	aesKey := keySplit.Sum(nil)
	keySplit.Reset()
	keySplit.Write([]byte("AES-IV"))
	keySplit.Write(secret)
	aesIV := keySplit.Sum(nil)

	aesCipher, err := aes.NewCipher(aesKey[:16])
	if err != nil {
		return nil, err
	}
	return cipher.NewCTR(aesCipher, aesIV[:16]), nil
}

func signatureHashValue(client bool, hfunc crypto.Hash, localPublic [32]byte, remotePublic [32]byte) []byte {
	hash := hfunc.New()
	if client {
		hash.Write(remotePublic[:])
		hash.Write(localPublic[:])
	} else {
		hash.Write(localPublic[:])
		hash.Write(remotePublic[:])
	}
	return hash.Sum(nil)
}

func certificateParse(cert []byte) (*x509.Certificate, error) {
	bundle, err := pkcs7.Parse(cert)
	if err != nil {
		return nil, err
	}

	if len(bundle.Certificates) != 1 {
		return nil, fmt.Errorf("wrong number of certificates found: %d", len(bundle.Certificates))
	}

	return bundle.Certificates[0], nil
}

func signatureVerifyClient(cert *x509.Certificate, localPublic [32]byte, remotePublic [32]byte, signature []byte) error {
	hashed, err := signatureCreateHash(true, cert, localPublic, remotePublic)
	if err != nil {
		return err
	}

	return mfipki.SignatureValidate(cert.PublicKey, hashed, signature)
}

func signatureCreateHash(client bool, cert *x509.Certificate, localPublic [32]byte, remotePublic [32]byte) ([]byte, error) {
	var hash crypto.Hash
	switch publicKey := cert.PublicKey.(type) {
	case *rsa.PublicKey:
		hash = crypto.SHA1
	case *ecdsa.PublicKey:
		hash = crypto.SHA256
	default:
		return nil, fmt.Errorf("unsupported public key type %T", publicKey)
	}

	return signatureHashValue(client, hash, localPublic, remotePublic), nil
}
