/**
 * @module MobileMQTTHandler
 * @description Manages the WebSocket-based MQTT connection for the mobile sensor.
 * Handles incoming data streams and routes them to the primary SensorService.
 */

const mqtt = require('mqtt');
const sensorService = require('../services/sensorService');
require('dotenv').config();

/**
 * Connection Configuration
 * The client connects using the WebSocket address defined in environment variables.
 */
const BROKER_URL = process.env.MQTT_APPLICATION_ADDR_WS;

const connectionOptions = {
    // Generates a unique ID for the mobile client session
    clientId: `backend_mobile_${Math.random().toString(16).slice(2, 8)}`,
    clean: true,
    reconnectPeriod: 5000, // Automatic retry every 5 seconds on connection loss
};

/**
 * Initialize MQTT Client
 */
const client = mqtt.connect(BROKER_URL, connectionOptions);

/**
 * Event: Connection Success
 * Triggered once the client successfully establishes a link with the broker.
 */
client.on('connect', () => {
    console.log('✅ [MQTT MOBILE] Connected via WebSocket');

    // Definiamo i topic (assicurati di aggiungere MQTT_BACKUP_TOPIC nel tuo .env)
    const primaryTopic = process.env.MQTT_APPLICATION_TOPIC;
    const backupTopic = process.env.MQTT_BACKUP_TOPIC || 'airquality/backup';
    
    const topicsToSubscribe = [primaryTopic, backupTopic];

    client.subscribe(topicsToSubscribe, (err) => {
        if (err) {
            console.error(`❌ [MQTT ERROR] Subscription failed on ${BROKER_URL}:`, err.message);
        } else {
            console.log(`📡 [MQTT MOBILE] Subscribed to topics: ${topicsToSubscribe.join(', ')}`);
        }
    });
});

/**
 * Event: Message Received
 * Parses the incoming buffer into a JSON object and passes it to the business logic layer.
 * * @param {string} topic - The topic path where the message was published.
 * @param {Buffer} message - The raw binary payload.
 */
client.on('message', (topic, message) => {
    try {
        const rawData = JSON.parse(message.toString());
        
        // Log opzionale per distinguere i flussi in console
        if (topic === process.env.MQTT_BACKUP_TOPIC) {
            console.log(`📦 [BACKUP DATA] Received data from recovery topic`);
        }

        // Forward data to the service
        sensorService.processIncomingData(rawData);
        
    } catch (error) {
        console.error(`⚠️ [MQTT PARSE ERROR] Failed to decode message on ${topic}:`, error.message);
    }
});

/**
 * Event: Error
 * Handles network-level errors or protocol violations.
 */
client.on('error', (err) => {
    console.error('🔌 [MQTT FATAL ERROR]:', err.message);
});

/**
 * Event: Reconnect
 * Logs attempts to recover a lost connection.
 */
client.on('reconnect', () => {
    console.warn('🔄 [MQTT MOBILE] Connection lost. Retrying...');
});

/**
 * @exports client
 * @description The active MQTT client instance.
 */
module.exports = client;