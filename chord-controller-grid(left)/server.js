const http = require('http');
const fs = require('fs');
const path = require('path');
const { WebSocketServer } = require('ws');

const PORT = 8082; // Use a different port to avoid conflicts if they run both
const PUBLIC_DIR = path.join(__dirname, 'public');

const MIME_TYPES = {
  '.html': 'text/html',
  '.css': 'text/css',
  '.js': 'application/javascript',
  '.json': 'application/json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.svg': 'image/svg+xml',
};

const server = http.createServer((req, res) => {
  let filePath = req.url === '/' ? '/index.html' : req.url;
  filePath = path.join(PUBLIC_DIR, filePath);

  const ext = path.extname(filePath);
  const contentType = MIME_TYPES[ext] || 'text/plain';

  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404);
      res.end('Not found');
      return;
    }
    res.writeHead(200, { 'Content-Type': contentType });
    res.end(data);
  });
});

const wss = new WebSocketServer({ server });

wss.on('connection', (ws, req) => {
  const ip = req.socket.remoteAddress;
  console.log(`[WS] Client connected from ${ip}`);

  ws.on('message', (data) => {
    try {
      const msg = JSON.parse(data.toString());
      console.log('[CHORD]', msg);
      // Broadcast chord to all other clients (e.g. midi.html)
      wss.clients.forEach((client) => {
        if (client !== ws && client.readyState === 1) {
          client.send(JSON.stringify(msg));
        }
      });
    } catch {
      console.log('[RAW]', data.toString());
    }
  });

  ws.on('close', () => console.log('[WS] Client disconnected'));
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`HTTP:  http://0.0.0.0:${PORT}`);
  console.log(`WS:    ws://0.0.0.0:${PORT}`);
  console.log('Open on phone: http://<YOUR_LAN_IP>:8082');
});
