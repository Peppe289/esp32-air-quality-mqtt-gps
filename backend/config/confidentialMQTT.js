/**
 * @module ConfidentialMQTTHandler
 * @description Establishes and manages the connection to the confidential MQTT broker 
 * for receiving fixed station environmental data.
 */

const mqtt = require('mqtt');
const sensorService = require('../services/sensorServiceConfidential');
require('dotenv').config();

/**
 * Broker Connection Configuration
 * Uses environment variables for sensitive credentials and network addresses.
 */
const connectionUrl = `mqtt://${process.env.MQTT_CONFIDENTIALS_WS}:${process.env.MQTT_CONFIDENTIALS_PORT}`;

const connectionOptions = {
    username: process.env.MQTT_CONFIDENTIALS_USERNAME,
    password: process.env.MQTT_CONFIDENTIALS_PASSWORD,
    // Cleanly identifies this client to the broker
    clientId: `backend_confidential_${Math.random().toString(16).slice(2, 8)}`,
    clean: true,
    reconnectPeriod: 5000, // Attempt reconnection every 5 seconds
};

/**
 * Initialize MQTT Client
 */
const confidentialMQTT = mqtt.connect(connectionUrl, connectionOptions);

/**
 * Event: Connection Success
 * Triggered when the client successfully authenticates with the broker.
 */
confidentialMQTT.on('connect', () => {
    console.log('✅ [MQTT CONFIDENTIAL] Connected successfully');

    // Subscribe to all subtopics (Wildcard)
    const targetTopic = process.env.MQTT_CONFIDENTIALS_TOPIC || '#'; 

    confidentialMQTT.subscribe(targetTopic, (err) => {
        if (err) {
            console.error(`❌ [MQTT ERROR] Subscription failed on ${process.env.MQTT_CONFIDENTIALS_WS}:`, err.message);
        } else {
            console.log(`📡 [MQTT CONFIDENTIAL] Subscribed to topic: ${targetTopic}`);
        }
    });
});

/**
 * Event: Message Received
 * Handles incoming data packets, parses the JSON payload, and forwards it to the service layer.
 * * @param {string} topic - The specific topic the message was published to.
 * @param {Buffer} message - The raw binary payload from the broker.
 */
confidentialMQTT.on('message', (topic, message) => {
    try {
        const rawData = JSON.parse(message.toString());
        
        // Pass data to the dedicated service for normalization and persistence
        sensorService.processConfidentialData(rawData);

    } catch (error) {
        console.error(`⚠️ [MQTT PARSE ERROR] Invalid JSON received on topic ${topic}:`, error.message);
    }
});

/**
 * Event: Connection Error
 * Triggered by network failures or authentication issues.
 */
confidentialMQTT.on('error', (err) => {
    console.error('🔌 [MQTT FATAL ERROR]:', err.message);
});

/**
 * Event: Reconnect
 * Triggered when the client attempts to re-establish a dropped connection.
 */
confidentialMQTT.on('reconnect', () => {
    console.warn('🔄 [MQTT CONFIDENTIAL] Attempting to reconnect...');
});

module.exports = confidentialMQTT;