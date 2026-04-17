/**
 * @module SensorServiceConfidential
 * @description Handles business logic and persistence for fixed environmental stations 
 * connected via the confidential MQTT broker.
 */

const Database = require('better-sqlite3');
const path = require('path');

/** * --- DATABASE INITIALIZATION ---
 * Connects to the fixed stations database. 
 * Ensure the 'data' directory exists in the project root.
 */
const dbPath = path.resolve(__dirname, '../data/stazioni_fisse.db');
const db_stazioni = new Database(dbPath);

/**
 * Initialize Table Schema
 */
db_stazioni.exec(`
    CREATE TABLE IF NOT EXISTS letture_stazioni_fisse (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        station_id TEXT,
        orario TEXT,
        pm1_0 REAL,
        pm2_5 REAL,
        pm10 REAL,
        temperatura REAL,
        umidita REAL,
        voc INTEGER,
        dispositivi_rilevati INTEGER
    )
`);

const SensorServiceConfidential = {
    /**
     * Processes incoming MQTT payloads from fixed stations.
     * Converts Unix timestamps to ISO strings and flattens the data structure.
     * * @param {Object} rawData - The raw JSON payload from the broker.
     * @param {string} rawData.ID - Unique station identifier.
     * @param {number} rawData.timestamp - Unix timestamp (seconds).
     * @param {number} [rawData.pm2_5] - PM2.5 concentration.
     * @param {number} [rawData.temperatura] - Ambient temperature.
     */
    async processConfidentialData(rawData) {
        try {
            // 1. Time Normalization (Unix seconds to ISO string)
            const orarioISO = new Date(rawData.timestamp * 1000).toISOString();

            // 2. Data Flattening & Mapping
            const flatData = {
                station_id: rawData.ID,
                orario: orarioISO,
                pm1_0: rawData.pm1 || 0,
                pm2_5: rawData.pm2_5 || 0,
                pm10: rawData.pm10 || 0,
                temperatura: rawData.temperatura || 0,
                umidita: rawData.umidita || 0,
                voc: rawData.voc_index || 0,
                dispositivi_rilevati: rawData.num_devices_sniffed || 0
            };

            // 3. Database Persistence
            this.saveConfidentialToDb(flatData);

            // 4. Debug Logging
            console.log(`💾 [CONFIDENTIAL DB] Record saved for: ${flatData.station_id} @ ${flatData.orario}`);

        } catch (err) {
            console.error(`❌ [ERROR] Confidential data processing failed: ${err.message}`);
        }
    },

    /**
     * Executes the SQL Insert statement for fixed station readings.
     * * @param {Object} data - The normalized data object.
     * @throws {Error} If the database write operation fails.
     */
    saveConfidentialToDb(data) {
        try {
            const query = `
                INSERT INTO letture_stazioni_fisse 
                (station_id, orario, pm1_0, pm2_5, pm10, temperatura, umidita, voc, dispositivi_rilevati)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            `;
            const stmt = db_stazioni.prepare(query);

            stmt.run(
                data.station_id, 
                data.orario, 
                data.pm1_0, 
                data.pm2_5, 
                data.pm10, 
                data.temperatura, 
                data.umidita, 
                data.voc, 
                data.dispositivi_rilevati
            );
        } catch (err) {
            console.error(`❌ [DB ERROR] Failed to write fixed station data: ${err.message}`);
        }
    },

    /**
     * Retrieves historical records for fixed stations with optional filtering.
     * * @param {Object} filters - Query parameters.
     * @param {string} [filters.station_id] - Filter by specific station ID.
     * @param {string} [filters.inizio] - Start ISO date string.
     * @param {string} [filters.fine] - End ISO date string.
     * @param {number} [filters.limit=100] - Result set limit.
     * @returns {Array<Object>} List of fixed station readings.
     */
    getConfidentialHistory({ station_id, inizio, fine, limit = 100 }) {
        let query = "SELECT * FROM letture_stazioni_fisse";
        const params = [];
        const conditions = [];

        if (station_id) {
            conditions.push("station_id = ?");
            params.push(station_id);
        }
        
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

        query += " ORDER BY orario DESC LIMIT ?";
        params.push(parseInt(limit));

        try {
            return db_stazioni.prepare(query).all(...params);
        } catch (err) {
            console.error(`❌ [DB ERROR] History fetch failed: ${err.message}`);
            return [];
        }
    }
};

module.exports = SensorServiceConfidential;