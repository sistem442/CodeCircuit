#include <Arduino.h>
String webpageCode = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Sensordaten</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 0;
    }
    h1 {
      background-color: #4CAF50;
      color: white;
      padding: 10px;
    }
    .status {
      display: inline-block;
      padding: 15px;
      margin: 10px;
      border-radius: 5px;
      color: white;
      font-weight: bold;
      width: 150px;
    }
    .status.green {
      background-color: #4CAF50;
    }
    .status.red {
      background-color: #f44336;
    }
    .controls {
      margin: 20px 0;
    }
    button {
      padding: 10px 20px;
      margin: 10px;
      font-size: 16px;
      cursor: pointer;
    }
    input[type="range"] {
      width: 80%;
    }
    .data {
      margin-top: 20px;
    }
    .data p {
      font-size: 18px;
      margin: 5px 0;
    }
    .mode {
      margin: 20px;
      padding: 10px;
      font-size: 18px;
      color: white;
      background-color: #2196F3;
      border-radius: 5px;
      display: inline-block;
    }
  </style>
</head>
<body>
  <h1>ESP32 Sensordaten</h1>

  <!-- Steuerungsmodus -->
  <div id="control_mode" class="mode">Modus: --</div>

  <!-- Motorstatus -->
  <div>
    <div id="motor_status" class="status">Motorstatus: --</div>
    <div id="error_status" class="status">Fehlerstatus: --</div>
  </div>

  <!-- Steuerungselemente -->
  <div class="controls">
    <button onclick="sendCommand('start_stop')">Start/Stop</button>
    <button onclick="sendCommand('change_direction')">Richtungswechsel</button>
    <button onclick="sendCommand('clear_error')">Fehler bestätigen</button>
    <div>
      <label for="speed_slider">Geschwindigkeit:</label>
      <input id="speed_slider" type="range" min="0" max="255" value="0" oninput="updateSpeed(this.value)" />
      <span id="speed_value">0</span>
    </div>
  </div>

  <!-- Sensordaten -->
  <div class="data">
    <p>Temperatur: <span id="temperature">--</span> °C</p>
    <p>Strom: <span id="current">--</span> mA</p>
    <p>Vibration: <span id="vibration">--</span></p>
    <p>Richtung: <span id="direction">--</span></p>
  </div>

  <script>
    const ws = new WebSocket('ws://' + location.hostname + ':81/');

    ws.onopen = () => console.log('WebSocket verbunden');
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        // Sensordaten aktualisieren
        document.getElementById('temperature').textContent = data.temperature.toFixed(2);
        document.getElementById('current').textContent = data.current.toFixed(2);
        document.getElementById('vibration').textContent = data.vibration ? 'Ja' : 'Nein';
        document.getElementById('direction').textContent = data.direction || '--';

        // Motorstatus aktualisieren
        const motorStatus = document.getElementById('motor_status');
        motorStatus.textContent = `Motorstatus: ${data.motor_status}`;
        motorStatus.className = `status ${data.motor_status === 'Running' ? 'green' : 'red'}`;

        // Fehlerstatus aktualisieren
        const errorStatus = document.getElementById('error_status');
        errorStatus.textContent = `Fehlerstatus: ${data.error_message || 'Keine'}`;
        errorStatus.className = `status ${data.error_message ? 'red' : 'green'}`;

        // Geschwindigkeit aktualisieren
        document.getElementById('speed_value').textContent = data.speed || '0';
        document.getElementById('speed_slider').value = data.speed || '0';

        // Modus aktualisieren
        const controlMode = document.getElementById('control_mode');
        controlMode.textContent = `Modus: ${data.control_mode || '--'}`;
        controlMode.style.backgroundColor = data.control_mode === 'Physical' ? '#FF9800' : '#2196F3';
      } catch (error) {
        console.error('Fehler beim Verarbeiten der Nachricht:', error);
      }
    };
    ws.onclose = () => console.log('WebSocket geschlossen');

    function sendCommand(command) {
      const message = JSON.stringify({ command });
      ws.send(message);
    }

    function updateSpeed(value) {
      document.getElementById('speed_value').textContent = value;
      const message = JSON.stringify({ command: 'set_speed', speed: parseInt(value, 10) });
      ws.send(message);
    }
  </script>
</body>
</html>
)rawliteral";
