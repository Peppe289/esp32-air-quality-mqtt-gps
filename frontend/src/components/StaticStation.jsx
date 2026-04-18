import React, { useState, useEffect, useCallback } from 'react';
import { toast } from 'react-toastify';

const StaticStation = ({ setStaticStation, loading, setLoading}) => {
    const [data, setData] = useState([]);
    
    const [filters, setFilters] = useState({
        station_id: '',
        limit: 100
    });

    const VPN_Addr = import.meta.env.VITE_REACT_APP_VPN_ADDR || "http://localhost:5000";

    useEffect(() => {
        fetch(`${VPN_Addr}/api/static-station`)
            .then((response) => {
                if (response.ok) {
                    toast.success('Dati stazioni fisse caricati con successo!');
                    return response.json();
                } else {
                    toast.error('Errore nel caricamento dei dati delle stazioni fisse. Assicurati di essere connesso alla VPN aziendale.');
                    throw new Error('Errore nella richiesta: ' + response.status);
                }
            })
            .then((data) => {
                setStaticStation(data);
            })
            .catch((error) => {
                console.error('Errore durante il fetch:', error);
            });
    }, [VPN_Addr, setStaticStation]);

    const fetchData = useCallback(async () => {
        try {
            const queryParams = new URLSearchParams({
                station_id: filters.station_id,
                limit: filters.limit
            }).toString();

            const response = await fetch(`${VPN_Addr}/api/history-stazioni?${queryParams}`);
            if (!response.ok) throw new Error(`HTTP Error: ${response.status}`);

            if (response.status === 403) {
                toast.error('Accesso negato. Assicurati di essere connesso alla VPN aziendale.');
                
                return;
            }

            const rawResponse = await response.json();

            // Log critico: controlla la console del browser per vedere la struttura
            console.log("Dati ricevuti dal server:", rawResponse);

            // Gestione flessibile della risposta (array diretto o oggetto avvolto)
            const finalData = Array.isArray(rawResponse)
                ? rawResponse
                : (rawResponse.data || rawResponse.results || []);

            setData(finalData);
        } catch (error) {
            console.error("❌ Errore Fetch:", error);
            // Non resettiamo i dati per evitare che la tabella sparisca in caso di micro-interruzione VPN
        } finally {
            setLoading(false);
        }
    }, [filters.station_id, filters.limit, VPN_Addr, setLoading]);

    useEffect(() => {
        fetchData();
        const interval = setInterval(fetchData, 1000);
        return () => clearInterval(interval);
    }, [fetchData]);

    // Funzione helper per estrarre valori indipendentemente dal nome della chiave (case-insensitive)
    const getVal = (obj, keys) => {
        for (let key of keys) {
            if (obj[key] !== undefined && obj[key] !== null) return obj[key];
        }
        return null;
    };

    const formatNum = (val) => {
        const n = parseFloat(val);
        return isNaN(n) ? "--" : n.toFixed(1);
    };

    return (
        <div style={{ padding: '20px', backgroundColor: '#f9f9f9', minHeight: '100vh' }}>
            <div style={containerStyle}>
                <div style={headerStyle}>
                    <h2 style={{ margin: 0, color: '#2c3e50' }}>📊 Monitoraggio Stazioni Fisse</h2>
                    <div style={{ display: 'flex', gap: '10px' }}>
                        <input
                            placeholder="Cerca ID Stazione..."
                            value={filters.station_id}
                            onChange={(e) => setFilters(prev => ({ ...prev, station_id: e.target.value }))}
                            style={inputStyle}
                        />
                        <button onClick={fetchData} style={btnStyle}>🔄 Aggiorna</button>
                    </div>
                </div>

                <div style={{ overflowX: 'auto' }}>
                    <table style={{ width: '100%', borderCollapse: 'collapse' }}>
                        <thead>
                            <tr style={{ backgroundColor: '#34495e', color: '#fff' }}>
                                <th style={thStyle}>ID Stazione</th>
                                <th style={thStyle}>Orario</th>
                                <th style={thStyle}>PM 1.0</th>
                                <th style={thStyle}>PM 2.5</th>
                                <th style={thStyle}>PM 10</th>
                                <th style={thStyle}>Temp</th>
                                <th style={thStyle}>Dispositivi</th>
                            </tr>
                        </thead>
                        <tbody>
                            {data.length > 0 ? data.map((row, index) => (
                                <tr key={row.id || index} style={trStyle}>
                                    {/* Mappatura intelligente delle colonne */}
                                    <td style={tdStyle}>
                                        <span style={idBadge}>{getVal(row, ['station_id', 'ID', 'station_ID']) || 'N/D'}</span>
                                    </td>
                                    <td style={tdStyle}>
                                        {row.orario ? new Date(row.orario).toLocaleTimeString() : '--:--'}
                                    </td>
                                    <td style={{ ...tdStyle, fontWeight: 'bold', color: '#e67e22' }}>
                                        {formatNum(getVal(row, ['pm1_0', 'pm1', 'PM1.0']))}
                                    </td>
                                    <td style={{ ...tdStyle, fontWeight: 'bold', color: '#e67e22' }}>
                                        {formatNum(getVal(row, ['pm2_5', 'pm25', 'PM2.5']))}
                                    </td>
                                    <td style={{ ...tdStyle, fontWeight: 'bold', color: '#d35400' }}>
                                        {formatNum(getVal(row, ['pm10', 'PM10']))}
                                    </td>
                                    <td style={tdStyle}>
                                        {formatNum(getVal(row, ['temperatura', 'temp', 'temperature']))}°C
                                    </td>
                                    <td style={tdStyle}>
                                        {getVal(row, ['dispositivi_rilevati', 'num_devices_sniffed', 'dispositivi']) ?? '--'}
                                    </td>
                                </tr>
                            )) : (
                                <tr>
                                    <td colSpan="7" style={{ textAlign: 'center', padding: '40px', color: '#7f8c8d' }}>
                                        {loading ? "📡 In attesa di dati dalla VPN..." : "📭 Nessun dato trovato."}
                                    </td>
                                </tr>
                            )}
                        </tbody>
                    </table>
                </div>
            </div>

            {/* Footer di Debug: utile per te per vedere l'ultimo oggetto ricevuto */}
            {data.length > 0 && (
                <div style={debugFooter}>
                    <strong>Debug Ultimo Record:</strong> {JSON.stringify(data[0]).substring(0, 100)}...
                </div>
            )}
        </div>
    );
};

// --- STILI ---
const containerStyle = { backgroundColor: '#fff', borderRadius: '12px', boxShadow: '0 4px 20px rgba(0,0,0,0.08)', overflow: 'hidden' };
const headerStyle = { padding: '20px', display: 'flex', justifyContent: 'space-between', alignItems: 'center', borderBottom: '1px solid #eee' };
const thStyle = { padding: '15px', textAlign: 'left', fontSize: '12px', letterSpacing: '1px' };
const tdStyle = { padding: '15px', fontSize: '14px', color: '#444' };
const trStyle = { borderBottom: '1px solid #f1f1f1' };
const inputStyle = { padding: '10px', borderRadius: '6px', border: '1px solid #ddd', outline: 'none', width: '200px' };
const btnStyle = { padding: '10px 20px', cursor: 'pointer', backgroundColor: '#3498db', color: 'white', border: 'none', borderRadius: '6px', transition: '0.2s' };
const idBadge = { backgroundColor: '#ecf0f1', padding: '4px 8px', borderRadius: '4px', fontSize: '12px', fontWeight: 'bold', color: '#2c3e50' };
const debugFooter = { marginTop: '20px', fontSize: '10px', color: '#bdc3c7', textAlign: 'center', fontSpace: 'monospace' };

export default StaticStation;