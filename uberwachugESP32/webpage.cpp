//============
//Webpage Code
//============
#include <Arduino.h>
String webpageCode = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Sensordaten</title>
</head>
<body>
  <h1>ESP32 Sensordaten</h1>
  <div>
    <p>Temperatur: <span id="temperature">--</span> °C</p>
    <p>Strom: <span id="current">--</span> A</p>
    <p>Vibration: <span id="vibration">--</span></p>
    <p>Motorstatus: <span id="motor_status">--</span></p>
    <p>Fehler: <span id="error_message">--</span></p>
    <p>Geschwindigkeit: <span id="speed">--</span></p>
    <p>Richtung: <span id="direction">--</span></p>
  </div>
  <script>
    const ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = () => console.log('WebSocket verbunden');
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        // Sensordaten
        document.getElementById('temperature').textContent = data.temperature.toFixed(2);
        document.getElementById('current').textContent = data.current.toFixed(2);
        document.getElementById('vibration').textContent = data.vibration ? 'Ja' : 'Nein';

        // Motorstatus und Fehler
        document.getElementById('motor_status').textContent = data.motor_status;
        document.getElementById('error_message').textContent = data.error_message || 'Keine';

        // Geschwindigkeit und Richtung
        document.getElementById('speed').textContent = data.speed || '--';
        document.getElementById('direction').textContent = data.direction || '--';
      } catch (error) {
        console.error('Fehler beim Verarbeiten der Nachricht:', error);
      }
    };
    ws.onclose = () => console.log('WebSocket geschlossen');
  </script>
</body>
</html>

)rawliteral";