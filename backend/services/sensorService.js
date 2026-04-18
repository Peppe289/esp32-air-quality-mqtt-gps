/**
 * @module SensorService
 * @description Provides business logic for parsing, validating, and persisting mobile sensor data.
 */

const db = require('../config/database');

/**
 * Converts NMEA coordinate format (Degrees, Minutes, Cardinal) to Decimal Degrees.
 * * @param {number} deg - Degrees component of the coordinate.
 * @param {number} min - Minutes component of the coordinate.
 * @param {string} card - Cardinal direction (e.g., 'N', 'S', 'E', 'W').
 * @returns {number} The coordinate in decimal degrees, rounded to 6 decimal places.
 */
const toDecimal = (deg, min, card) => {
    // Basic type validation to prevent calculation errors
    if (typeof deg !== 'number' || typeof min !== 'number') return 0;
    
    let decimal = deg + (min / 60);
    
    // Convert to negative if Southern or Western hemisphere
    if (['S', 'W'].includes(card?.toUpperCase())) {
        decimal = -decimal;
    }
    
    // Standard 6 decimal places for GPS precision (~11cm accuracy)
    return parseFloat(decimal.toFixed(6));
};

/**
 * Core service object for sensor data management.
 */
const SensorService = {
    /**
     * Processes raw MQTT messages from the mobile sensor firmware.
     * Normalizes data and enforces GPS quality gates before database insertion.
     * * @param {Object} rawData - Raw JSON payload received from the sensor.
     */
    processIncomingData(rawData) {
        try {
            // Coordinate extraction using optional chaining for safety
            const lat = toDecimal(
                rawData.position?.latitude?.degrees, 
                rawData.position?.latitude?.minutes, 
                rawData.position?.latitude?.cardinal
            );
            const lon = toDecimal(
                rawData.position?.longitude?.degrees, 
                rawData.position?.longitude?.minutes, 
                rawData.position?.longitude?.cardinal
            );

            // Handle timestamp: fallback to server time if firmware timestamp is missing
            const timestamp = rawData.orario || new Date().toISOString();

            // Construct normalized data object (flat structure)
            const sensorReading = {
                timestamp,
                lat,
                lon,
                pm1_0: rawData.hm3301?.["PM1.0"] || 0,
                pm2_5: rawData.hm3301?.["PM2.5"] || 0,
                pm10: rawData.hm3301?.["PM10"] || 0,
                satellites: rawData.satellites || 0
            };

            /** * GPS Quality Gate:
             * Data is only saved if coordinates are non-zero AND 
             * at least 3 satellites are locked for reliability.
             */
            const hasValidGpsFix = lat !== 0 && lon !== 0 && sensorReading.satellites >= 3;

            if (hasValidGpsFix) {
                this.saveToDb(sensorReading);
            } else {
                console.warn(`📡 [SKIP] Weak GPS Signal: ${sensorReading.satellites} satellites.`);
            }
        } catch (error) {
            console.error(`❌ [ERROR] Failed to process sensor data: ${error.message}`);
        }
    },

    /**
     * Executes the SQL query to persist data into the SQLite database.
     * * @param {Object} data - Normalized sensor data object.
     * @throws {Error} If the database operation fails.
     */
    saveToDb(data) {
        try {
            const query = `
                INSERT OR IGNORE INTO letture (orario, lat, lon, pm1_0, pm2_5, pm10, satelliti)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            `;
            const stmt = db.prepare(query);
            
            const result = stmt.run(
                data.timestamp, 
                data.lat, 
                data.lon, 
                data.pm1_0, 
                data.pm2_5, 
                data.pm10, 
                data.satellites
            );

            if (result.changes > 0) {
                console.log(`💾 [DB] Data persisted: ${data.timestamp}`);
            }
        } catch (err) {
            console.error(`❌ [DB ERROR] Write failed: ${err.message}`);
            // Re-throw if you want the caller to handle higher-level logic
        }
    },

    /**
     * Retrieves historical sensor records based on time filters.
     * * @param {Object} filters - Search parameters.
     * @param {string} [filters.inizio] - Start ISO string (inclusive).
     * @param {string} [filters.fine] - End ISO string (inclusive).
     * @param {number} [filters.limit=5000] - Max number of records to return.
     * @returns {Array<Object>} List of sensor readings.
     */
    getHistory({ inizio, fine, limit = 5000 }) {
        let query = "SELECT * FROM letture";
        const params = [];
        const conditions = [];

        if (inizio) {
            conditions.push("orario >= ?");
            params.push(inizio);
        }
        if (fine) {
            conditions.push("orario <= ?");
            params.push(fine);
        }

        if (conditions.length > 0) {
            query += " WHERE " + conditions.join(" AND ");
        }

        query += " ORDER BY orario ASC LIMIT ?";
        params.push(parseInt(limit));

        try {
            return db.prepare(query).all(...params);
        } catch (err) {
            console.error(`❌ [DB ERROR] History query failed: ${err.message}`);
            return [];
        }
    }
};

module.exports = SensorService;