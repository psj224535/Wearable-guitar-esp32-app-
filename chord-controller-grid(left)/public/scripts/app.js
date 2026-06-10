(() => {
  'use strict';

  // Chords mapping to index: C=0, D7=1, Em=2, G=3, A=4, F=5
  const CHORDS = ['C', 'D7', 'Em', 'G', 'A', 'F'];

  const WS_HOST = new URLSearchParams(location.search).get('host')
    || location.hostname
    || 'localhost';
  const WS_PORT = new URLSearchParams(location.search).get('port') || '8082';
  const WS_URL = `ws://${WS_HOST}:${WS_PORT}`;

  const activeTouches = new Map();
  const chordPressCount = new Map();

  let ws = null;
  let reconnectTimer = null;
  let shouldReconnect = true;

  const statusDot = document.getElementById('statusDot');
  const statusText = document.getElementById('statusText');
  const chordGrid = document.getElementById('chordGrid');

  const buttonByChord = new Map();

  CHORDS.forEach((chord) => {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.dataset.chord = chord;
    btn.className = [
      'chord-btn flex h-full w-full flex-col items-center justify-center',
      'rounded-2xl border-2 border-white/10 bg-panel',
      'text-5xl font-black tracking-tight sm:text-6xl',
      'focus:outline-none focus-visible:ring-2 focus-visible:ring-accent/50',
    ].join(' ');
    btn.innerHTML = `
      <span class="leading-none text-neon drop-shadow-[0_0_12px_rgba(57,255,20,0.45)]">${chord}</span>
      <span class="mt-1 text-xs font-semibold uppercase tracking-[0.2em] text-white/35">Major</span>
    `;
    chordGrid.appendChild(btn);
    buttonByChord.set(chord, btn);
  });

  function setConnectionStatus(state, label) {
    statusDot.className = `status-dot ${state}`;
    statusText.textContent = label;
  }

  function updateButtonVisual(chord) {
    const btn = buttonByChord.get(chord);
    const count = chordPressCount.get(chord) || 0;
    btn?.classList.toggle('active', count > 0);
  }

  function sendPayload(chord, action) {
    const payload = { chord, action };
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(payload));
      console.log('[SEND]', payload);
    } else {
      console.warn('[WS offline]', payload);
    }
  }

  function pressChord(chord) {
    const count = chordPressCount.get(chord) || 0;
    if (count === 0) {
      sendPayload(chord, 'press');
    }
    chordPressCount.set(chord, count + 1);
    updateButtonVisual(chord);
  }

  function releaseChord(chord) {
    const count = chordPressCount.get(chord) || 0;
    if (count <= 1) {
      chordPressCount.delete(chord);
      sendPayload(chord, 'release');
    } else {
      chordPressCount.set(chord, count - 1);
    }
    updateButtonVisual(chord);
  }

  function getButtonFromPoint(x, y) {
    const el = document.elementFromPoint(x, y);
    return el?.closest('[data-chord]') || null;
  }

  function onTouchStart(e) {
    e.preventDefault();
    for (const touch of e.changedTouches) {
      const btn = getButtonFromPoint(touch.clientX, touch.clientY);
      if (!btn) continue;

      const chord = btn.dataset.chord;
      activeTouches.set(touch.identifier, { chord, buttonEl: btn });
      pressChord(chord);
    }
  }

  function onTouchMove(e) {
    e.preventDefault();
    for (const touch of e.changedTouches) {
      const prev = activeTouches.get(touch.identifier);
      if (!prev) continue;

      const btn = getButtonFromPoint(touch.clientX, touch.clientY);
      const newChord = btn?.dataset.chord;

      if (newChord && newChord !== prev.chord) {
        releaseChord(prev.chord);
        activeTouches.set(touch.identifier, { chord: newChord, buttonEl: btn });
        pressChord(newChord);
      } else if (!newChord) {
        releaseChord(prev.chord);
        activeTouches.delete(touch.identifier);
      }
    }
  }

  function onTouchEndOrCancel(e) {
    e.preventDefault();
    for (const touch of e.changedTouches) {
      const prev = activeTouches.get(touch.identifier);
      if (!prev) continue;

      releaseChord(prev.chord);
      activeTouches.delete(touch.identifier);
    }
  }

  const touchTarget = document.getElementById('app');
  touchTarget.addEventListener('touchstart', onTouchStart, { passive: false });
  touchTarget.addEventListener('touchmove', onTouchMove, { passive: false });
  touchTarget.addEventListener('touchend', onTouchEndOrCancel, { passive: false });
  touchTarget.addEventListener('touchcancel', onTouchEndOrCancel, { passive: false });

  touchTarget.addEventListener('mousedown', (e) => e.preventDefault());
  touchTarget.addEventListener('contextmenu', (e) => e.preventDefault());

  function connectWebSocket() {
    if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    setConnectionStatus('connecting', 'Connecting…');
    ws = new WebSocket(WS_URL);

    ws.addEventListener('open', () => {
      setConnectionStatus('connected', 'Connected');
    });

    ws.addEventListener('close', () => {
      setConnectionStatus('disconnected', 'Offline');
      ws = null;
      if (shouldReconnect) {
        clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(connectWebSocket, 1500);
      }
    });

    ws.addEventListener('error', () => {
      setConnectionStatus('disconnected', 'Error');
    });
  }

  connectWebSocket();

  window.addEventListener('beforeunload', () => {
    shouldReconnect = false;
    ws?.close();
  });
})();
