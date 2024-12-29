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
  <style>
    #error-message {
      display: none;
      color: red;
      font-weight: bold;
    }
  </style>
</head>
<body>
  <h1>ESP32 Sensordaten</h1>
  <div>
    <p>Temperatur: <span id="temperature">--</span> °C</p>
    <p>Strom: <span id="current">--</span> A</p>
    <p>Vibration: <span id="vibration">--</span></p>
  </div>
  <div>
    <p>Motorstatus: <span id="status-text">Unbekannt</span></p>
    <p id="error-message">Fehler: <span id="error-text"></span></p>
  </div>
  <script>
    const ws = new WebSocket('ws://' + location.hostname + ':81/');
    
    ws.onopen = () => console.log('WebSocket verbunden');
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        // Aktualisiere Sensordaten
        if (data.temperature !== undefined) {
          document.getElementById('temperature').textContent = data.temperature.toFixed(2);
        }
        if (data.current !== undefined) {
          document.getElementById('current').textContent = data.current.toFixed(2);
        }
        if (data.vibration !== undefined) {
          document.getElementById('vibration').textContent = data.vibration ? 'Ja' : 'Nein';
        }

        // Aktualisiere Motorstatus und Fehler
        if (data.motor_status !== undefined) {
          const statusText = document.getElementById("status-text");
          const errorDiv = document.getElementById("error-message");
          const errorText = document.getElementById("error-text");

          if (data.motor_status === "stopped") {
            statusText.textContent = "Gestoppt";
            statusText.style.color = "red";

            if (data.error_message) {
              errorDiv.style.display = "block";
              errorText.textContent = data.error_message;
            }
          } else if (data.motor_status === "running") {
            statusText.textContent = "Läuft";
            statusText.style.color = "green";
            errorDiv.style.display = "none";
          }
        }
      } catch (error) {
        console.error('Fehler beim Verarbeiten der Nachricht:', error);
      }
    };

    ws.onclose = () => console.log('WebSocket geschlossen');
  </script>
</body>
</html>
)rawliteral";