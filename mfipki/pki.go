package mfipki

import (
	"crypto"
	"crypto/ecdsa"
	"crypto/rsa"
	"errors"
	"fmt"
	"math/big"
)

func SignatureValidate(publicKey interface{}, hashed []byte, signature []byte) error {
	switch pkey := publicKey.(type) {
	case *rsa.PublicKey:
		return rsa.VerifyPKCS1v15(pkey, crypto.SHA1, hashed, signature)

	case *ecdsa.PublicKey:
		/* TODO: this is a guess, but it is the 'default' encoding */
		r := big.Int{}
		s := big.Int{}

		mid := len(signature) / 2
		r.SetBytes(signature[:mid])
		s.SetBytes(signature[mid:])

		if ecdsa.Verify(pkey, hashed, &r, &s) {
			return nil
		}

		return errors.New("ecdsa signature was invalid")

	default:
		return fmt.Errorf("unsupported signature algorithm: %T", pkey)
	}
}
