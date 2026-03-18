const net = require('net');

const server_port = 65432;
const server_addr = "192.168.0.209";

// This function sends a command to the Pi and updates the UI with the response
function sendCommand(command) {
    const client = net.createConnection({ port: server_port, host: server_addr }, () => {
        if (command !== 'status') {
            console.log(`Sending command: ${command}`);
        }
        client.write(command);
    });

    // Handle the data returned from the server
    client.on('data', (data) => {
        try {
            const carState = JSON.parse(data.toString());
            
            // Update the HTML elements with the new data
            document.getElementById("distance").innerHTML = carState.distance;
            document.getElementById("moving").innerHTML = carState.moving;
            document.getElementById("speed").innerHTML = carState.speed;
            
        } catch (e) {
            console.error("Error parsing JSON data from server: ", e);
        }
        
        client.end();
        client.destroy();
    });

    client.on('end', () => {
        // Connection closed
    });
    
    client.on('error', (err) => {
        console.error('Connection error (is the Python server running?): ', err);
    });
}

// Automatically ask the server for a 'status' update every second
setInterval(() => {
    sendCommand('status');
}, 1000);