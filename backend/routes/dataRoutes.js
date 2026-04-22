/**
 * @module Routes
 * @description API route definitions for mobile and fixed sensor data management.
 */

const express = require('express');
const router = express.Router();
const path = require('path');
const fs = require('fs');
const SensorService = require('../services/sensorService');
const SensorServiceConfidential = require('../services/sensorServiceConfidential');
const publicRoute = "/public";
const confidentialRoute = "/confidential";

/**
 * Global logging middleware for debugging incoming requests.
 */
router.use((req, res, next) => {
    console.log(`📥 Incoming Request: ${req.method} ${req.originalUrl}`, {
        query: req.query,
        ip: req.headers['x-forwarded-for'] || req.socket.remoteAddress
    });
    next();
});

/**
 * @route GET /api/history-stazioni
 * @group Confidential - Accessible via VPN only
 * @returns {Object} 200 - An object containing the array of records and total count.
 */
router.get(`${confidentialRoute}/history-stazioni`, (req, res) => {
    try {
        const filters = {
            station_id: req.query.station_id,
            limit: req.query.limit ? parseInt(req.query.limit) : 5000
        };

        const results = SensorServiceConfidential.getConfidentialHistory(filters);
        res.json({ results, count: results.length });
    } catch (err) {
        console.error(`❌ [ROUTE ERROR] /history-stazioni: ${err.message}`);
        res.status(500).json({ error: "Internal Server Error" });
    }
});

/**
 * @route GET /api/static-station
 * @description Serves the static configuration of fixed stations from a JSON file.
 * @group Confidential - Accessible via VPN only
 */
router.get(`${confidentialRoute}/static-station`, (req, res) => {
    const filePath = path.join(__dirname, '..', 'data', 'stazioni_fisse.json');
    
    fs.readFile(filePath, 'utf8', (err, data) => {
        if (err) {
            console.error(`❌ [FS ERROR] Failed to read ${filePath}:`, err.message);
            return res.status(500).json({ error: "Configuration file missing or unreadable" });
        }

        try {
            const jsonData = JSON.parse(data);
            res.json(jsonData);
        } catch (parseErr) {
            console.error(`❌ [JSON ERROR] Malformed file ${filePath}:`, parseErr.message);
            res.status(500).json({ error: "Invalid configuration format" });
        }
    });
});

router.get(`${confidentialRoute}/data`, (req, res) => {
    try {
        const { day, ...filters } = req.query;

        // If a specific day is provided, we override start/end filters to cover the full 24h
        if (day) {
            filters.inizio = `${day}T00:00:00.000Z`;
            filters.fine = `${day}T23:59:59.999Z`;
        }

        const results = SensorService.getHistory(filters);
        res.json({ results, count: results.length });
    } catch (err) {
        console.error(`❌ [ROUTE ERROR] /data: ${err.message}`);
        res.status(500).json({ error: "Internal Server Error" });
    }
});

/**
 * @route GET /api/data
 * @description Retrieves historical data for the mobile sensor.
 * @query {string} day - Optional specific day (YYYY-MM-DD)
 * @query {string} inizio - Optional ISO start date
 * @query {string} fine - Optional ISO end date
 * @group Public - Accessible within the local subnet
 */
router.get(`${publicRoute}/data`, (req, res) => {
    try {
        const { day, ...filters } = req.query;

        const clientIp = req.headers['x-forwarded-for'] || req.socket.remoteAddress;
        // if the request is not from the VPN, we apply a geofencing filter
        // to limit data to the area around the university
        console.log(`🌐 Applying geofencing filters for IP ${clientIp}`);
        filters.range_lat = {
            min: 40.7670,
            max: 40.7755
        };
        filters.range_lon = {
            min: 14.7850,
            max: 14.7965
        };

        // If a specific day is provided, we override start/end filters to cover the full 24h
        if (day) {
            filters.inizio = `${day}T00:00:00.000Z`;
            filters.fine = `${day}T23:59:59.999Z`;
        }

        const results = SensorService.getHistory(filters);
        res.json({ results, count: results.length });
    } catch (err) {
        console.error(`❌ [ROUTE ERROR] /data: ${err.message}`);
        res.status(500).json({ error: "Internal Server Error" });
    }
});

module.exports = router;