package waclib

import "errors"

type tlvField struct {
	Type uint8
	Data []byte
}

func tlvUnmarshal(input []byte) ([]tlvField, error) {
	var result []tlvField

	for {
		if len(input) == 0 {
			break
		}
		if len(input) < 2 {
			return nil, errors.New("malformed TLV message")
		}

		fieldLen := int(input[1])
		t := input[0]
		input = input[2:]
		if fieldLen > len(input) {
			return nil, errors.New("field truncated")
		}

		result = append(result, tlvField{
			Type: t,
			Data: append([]byte{}, input[:fieldLen]...),
		})

		input = input[fieldLen:]
	}

	return result, nil
}

func tlvMarshal(fields []tlvField) []byte {
	var result []byte

	for _, m := range fields {
		dl := len(m.Data)
		if dl > 255 {
			panic("Oversized TLV field")
		}
		result = append(result, []byte{m.Type, byte(dl)}...)
		result = append(result, m.Data...)
	}

	return result
}
