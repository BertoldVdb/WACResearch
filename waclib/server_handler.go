package waclib

import (
	"io"
	"io/ioutil"
	"net/http"
)

type wacHandlerFunc func(session *serverSession, in wacMessageInput) (wacMessageOutput, error)

func (s *WACServer) makeHandler(desiredState serverSessionState, encrypt bool, getMsg func() wacMessageInput, cb wacHandlerFunc) func(w http.ResponseWriter, r *http.Request) {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.Method != "POST" {
			http.Error(w, "Invalid method", http.StatusMethodNotAllowed)
			return
		}

		session, release := s.sessions.GetSessionHTTP(w, r, desiredState)
		if session == nil {
			return
		}
		defer release()

		var msg wacMessageInput
		if getMsg != nil {
			msg = getMsg()

			data, err := ioutil.ReadAll(io.LimitReader(r.Body, 8192))
			if err != nil {
				session.state = -1

				http.Error(w, err.Error(), http.StatusBadRequest)
			}

			if encrypt && session.stream != nil {
				session.stream.XORKeyStream(data, data)
			}

			if err := msg.Unmarshal(data); err != nil {
				session.state = -1

				http.Error(w, "Failed to decode request: "+err.Error(), http.StatusBadRequest)
				return
			}
		}

		out, err := cb(session, msg)
		if err != nil {
			session.state = -1

			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}

		if out != nil {
			data := out.Marshal()

			if encrypt && session.stream != nil {
				session.stream.XORKeyStream(data, data)
			}

			w.Header().Set("Content-Type", "application/octet-stream")
			w.Write(data)
		}
	}
}
