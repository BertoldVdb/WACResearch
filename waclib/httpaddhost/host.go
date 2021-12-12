// httpaddhost adds a dummy Host: header to http requests.
// This is needed because some version of iOS send HTTP/1.1 requests lacking this header,
// which the golang stdlib http server does not accept
package httpaddhost

import (
	"bytes"
	"errors"
	"net"
	"strings"
	"sync/atomic"
)

type HTTPAddHost struct {
	net.Listener
}

func Wrap(listener net.Listener) *HTTPAddHost {
	return &HTTPAddHost{
		Listener: listener,
	}
}

type wrapConn struct {
	net.Conn

	inHeader bool
	hasHost  bool
	hdrLine  []byte

	needFlush uint32

	readFirst bytes.Buffer
}

func (w *wrapConn) Read(data []byte) (int, error) {
readMore:
	if !w.inHeader {
		if w.readFirst.Len() > 0 {
			return w.readFirst.Read(data)
		}
	}

	n, err := w.Conn.Read(data)
	if err != nil {
		return 0, err
	}

	nf := atomic.SwapUint32(&w.needFlush, 0)
	if nf > 0 {
		w.inHeader = true
		w.hasHost = false
		w.hdrLine = w.hdrLine[:0]
		w.readFirst.Reset()
	}

	if !w.inHeader {
		return n, err
	}

	for i, m := range data[:n] {
		if len(w.hdrLine) > 8*1024 || w.readFirst.Len() > 64*1024 {
			w.Close()
			return 0, errors.New("oversized header received")
		}

		w.hdrLine = append(w.hdrLine, m)
		if m == '\n' {
			if strings.HasPrefix(strings.ToLower(string(w.hdrLine)), "host") {
				w.hasHost = true
			}

			if len(w.hdrLine) == 2 {
				if !w.hasHost {
					w.readFirst.Write([]byte("Host: dummy\r\n"))
				}

				w.inHeader = false
			}

			w.readFirst.Write(w.hdrLine)
			w.hdrLine = w.hdrLine[:0]

			if !w.inHeader {
				if i < n {
					w.readFirst.Write(data[i+1 : n])
				}
				break
			}
		}
	}

	goto readMore
}

func (w *wrapConn) Write(data []byte) (int, error) {
	atomic.StoreUint32(&w.needFlush, 1)

	return w.Conn.Write(data)
}

func (d *HTTPAddHost) Accept() (net.Conn, error) {
	conn, err := d.Listener.Accept()
	if err != nil {
		return conn, err
	}

	return &wrapConn{
		Conn:     conn,
		inHeader: true,
	}, nil
}
