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
 * Middleware to restrict access to company VPN IP ranges (10.6.0.x).
 * @param {Object} req - Express request object.
 * @param {Object} res - Express response object.
 * @param {Function} next - Express next middleware function.
 */
const vpnOnly = (req, res, next) => {
    const clientIp = req.headers['x-forwarded-for'] || req.socket.remoteAddress;
    
    // Check if the IP starts with the WireGuard VPN subnet prefix
    const isVpn = clientIp.startsWith('10.6.0.') || clientIp === '::1';

    if (isVpn) {
        next();
    } else {
        console.warn(`🛑 Unauthorized Access Blocked: IP ${clientIp}`);
        res.status(403).json({ 
            error: "Forbidden", 
            message: "Access granted only via Company VPN." 
        });
    }
};

/**
 * @route GET /api/history-stazioni
 * @group Confidential - Accessible via VPN only
 * @returns {Object} 200 - An object containing the array of records and total count.
 */
router.get('/history-stazioni', vpnOnly, (req, res) => {
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
router.get('/static-station', vpnOnly, (req, res) => {
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

/**
 * @route GET /api/data
 * @description Retrieves historical data for the mobile sensor.
 * @query {string} day - Optional specific day (YYYY-MM-DD)
 * @query {string} inizio - Optional ISO start date
 * @query {string} fine - Optional ISO end date
 * @group Public - Accessible within the local subnet
 */
router.get('/data', (req, res) => {
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

module.exports = router;