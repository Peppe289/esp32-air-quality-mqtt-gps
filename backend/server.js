/**
 * @module Server
 * @description Main entry point for the PM Monitoring Backend.
 * Initializes database connections, MQTT listeners, and the Express REST API.
 */

const express = require('express');
const cors = require('cors');
require('dotenv').config();

// Initialize MQTT Handlers (Implicitly starts broker connections)
require('./config/mqtt');
require('./config/confidentialMQTT');

// Import Routes
const dataRoutes = require('./routes/dataRoutes');

const app = express();
const PORT = process.env.PORT || 5000;

/**
 * Middleware Configuration
 */
app.use(cors()); // Enable Cross-Origin Resource Sharing
app.use(express.json()); // Parse incoming JSON payloads

/**
 * Route Mapping
 */
app.use('/api', dataRoutes);

/**
 * Server Execution
 * Listening on 0.0.0.0 to allow access from the local network and VPN.
 */
const server = app.listen(PORT, '0.0.0.0', () => {
    console.log(`
🚀 [SERVER] System online
📡 Network Access: http://10.6.0.3:${PORT} (VPN) or http://localhost:${PORT}
📂 API Base Path: /api
    `);
});

/**
 * Graceful Shutdown Logic
 * Ensures that the server closes connections before exiting.
 */
process.on('SIGINT', () => {
    console.log('🛑 [SERVER] Shutting down...');
    server.close(() => {
        console.log('✅ [SERVER] HTTP server closed.');
        process.exit(0);
    });
});