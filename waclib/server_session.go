package waclib

import (
	"crypto/cipher"
	"net/http"
	"strings"
	"sync"
	"time"
)

type serverSessionState int

type serverSession struct {
	sync.Mutex

	expiry       time.Time
	state        serverSessionState
	allowRestart bool

	stream cipher.Stream
}

type serverSessionManager struct {
	sync.Mutex
	single bool

	nextCleanup time.Time
	sessions    map[string]*serverSession
}

func newServerSessionManager(single bool) *serverSessionManager {
	return &serverSessionManager{
		sessions: make(map[string]*serverSession),
		single:   single,
	}
}

func (s *serverSessionManager) handleCleanup() {
	now := time.Now()

	if now.Before(s.nextCleanup) {
		return
	}

	s.nextCleanup = now.Add(10 * time.Second)

	for i, m := range s.sessions {
		if m.expiry.Before(now) {
			delete(s.sessions, i)
		}
	}
}

func (s *serverSessionManager) create(key string) *serverSession {
	s.handleCleanup()

	session := &serverSession{
		allowRestart: true,
	}

	s.sessions[key] = session

	return session
}

func (s *serverSessionManager) get(key string) *serverSession {
	s.handleCleanup()

	session := s.sessions[key]
	if session != nil {
		session.expiry = time.Now().Add(30 * time.Second)

	}
	return session
}

func (s *serverSessionManager) Get(key string, desiredState serverSessionState) *serverSession {
	if s.single {
		key = ""
	}

	s.Lock()
	defer s.Unlock()

	session := s.get(key)
	if desiredState == 0 {
		if session == nil || session.allowRestart {
			session = s.create(key)
		}
	}

	return session
}

func (s *serverSessionManager) GetSessionHTTP(w http.ResponseWriter, r *http.Request, desiredState serverSessionState) (*serverSession, func()) {
	key := r.RemoteAddr
	port := strings.LastIndex(key, ":")
	if port >= 0 {
		key = key[:port]
	}

	session := s.Get(key, desiredState)

	if session == nil {
		http.Error(w, "No session found", http.StatusBadRequest)
		return nil, nil
	}

	session.Lock()
	if session.state != desiredState {
		session.state = -1

		http.Error(w, "Session in invalid state", http.StatusBadRequest)
		session.Unlock()
		return nil, nil
	}

	return session, func() {
		session.Unlock()
	}
}
