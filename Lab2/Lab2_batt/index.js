const net = require('net');
const { PythonShell } = require('python-shell');

// Basic settings
const server_port = 65432;

// This function sends a command to the Pi and updates the UI with the response
function sendCommand(command) {
    const connTypeSelection = document.querySelector('input[name="conn_type"]:checked');
    const connType = connTypeSelection ? connTypeSelection.value : 'wifi';

    if (connType === 'wifi') {
        sendWiFiCommand(command);
    } else {
        sendBluetoothCommand(command);
    }
}

function sendWiFiCommand(command) {
    const server_addr = document.getElementById("pi_ip").value;
    const client = net.createConnection({ port: server_port, host: server_addr }, () => {
        if (command !== 'status') {
            console.log(`Sending command: ${command}`);
        }
        client.write(command);
    });

    client.on('data', (data) => {
        try {
            updateUI(data.toString());
        } catch (e) {
            console.error("Error parsing WiFi JSON: ", e);
        }
        client.end();
        client.destroy();
    });

    client.on('error', (err) => {
        console.error('WiFi Connection error: ', err);
    });
}

function sendBluetoothCommand(command) {
    const server_mac = document.getElementById("pi_mac").value;
    
    let options = {
        mode: 'text',
        pythonOptions: ['-u'], // get print results in real-time
        args: [command, server_mac]
    };

    PythonShell.run('bt_client_helper.py', options).then(messages => {
        // messages is an array of messages collected from the client's stdout
        if (messages && messages.length > 0) {
            try {
                // The helper prints the JSON string.
                updateUI(messages[0]);
            } catch (e) {
                console.error("Error parsing BT JSON: ", e);
            }
        }
    }).catch(err => {
        console.error('Bluetooth PythonShell error: ', err);
    });
}

function updateUI(jsonString) {
    const carState = JSON.parse(jsonString);
    if (carState.error) {
        console.error("Server Error: ", carState.error);
        return;
    }
    // Update the HTML elements with the new data
    document.getElementById("distance").innerHTML = carState.distance;
    document.getElementById("moving").innerHTML = carState.moving;
    document.getElementById("speed").innerHTML = carState.speed;
    document.getElementById("battery").innerHTML = carState.battery;
}

// Automatically poll for 'status' update
setInterval(() => {
    sendCommand('status');
}, 1000);