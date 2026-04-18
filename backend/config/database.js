/**
 * @module DatabaseConfig
 * @description Initializes and configures the SQLite database for mobile sensor data.
 * Uses better-sqlite3 for synchronous, high-performance database operations.
 */

const Database = require('better-sqlite3');
const path = require('path');

/**
 * Database File Path
 * Resolves the path to ensure the database is created in the correct directory
 * regardless of where the process is started.
 */
const DB_PATH = path.resolve(__dirname, '../pm_monitor.db');

/**
 * Database Instance Initialization
 * @type {Database.Database}
 */
const db = new Database(DB_PATH, { 
    // verbose: console.log // Uncomment for SQL query debugging
});

/**
 * Performance Optimization: Write-Ahead Logging (WAL)
 * WAL mode significantly improves concurrency and write performance,
 * which is critical for high-frequency MQTT data streams.
 */
db.pragma('journal_mode = WAL');

/**
 * Schema Initialization
 * Creates the 'letture' (readings) table if it does not exist.
 * * Fields:
 * - orario: ISO string timestamp (Primary Key to prevent duplicates)
 * - lat/lon: Decimal coordinates
 * - pmX_X: Particulate matter concentrations
 * - satelliti: GPS signal quality indicator
 */
db.exec(`
  CREATE TABLE IF NOT EXISTS letture (
    orario TEXT PRIMARY KEY,
    lat REAL NOT NULL, 
    lon REAL NOT NULL,
    pm1_0 REAL, 
    pm2_5 REAL, 
    pm10 REAL,
    satelliti INTEGER
  )
`);

/**
 * @exports db
 * @description A singleton database connection instance to be used across the application.
 */
module.exports = db;