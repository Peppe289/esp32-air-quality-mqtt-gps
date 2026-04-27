import React, { useState, useEffect, useCallback, useMemo } from 'react';
import { MdRefresh, MdSettingsInputAntenna, MdCheckBox, MdCheckBoxOutlineBlank, MdFilterList } from "react-icons/md";

const StaticStation = ({ selectedIds, setSelectedIds, setStaticStation, setLoading }) => {
    const [data, setData] = useState([]);
    const [showSelector, setShowSelector] = useState(false);

    const addr = import.meta.env.VITE_SERVER_ADDR || "";

    const fetchData = useCallback(() => {
        fetch(addr + `/api/confidential/history-stazioni?limit=100`)
            .then(response => {
                if (!response.ok) throw new Error("Errore rete");
                return response.json(); // Ritorna la Promise del JSON
            })
            .then(rawResponse => {
                // Ora rawResponse è l'oggetto/array reale
                setData(
                    Array.isArray(rawResponse) ? rawResponse : (rawResponse.data || rawResponse.results || [])
                );
            })
            .catch(err => console.error("Errore fetch:", err));
    }, [addr])

    useEffect(() => {
        const interval = setInterval(fetchData, 5000);
        return () => clearInterval(interval);
    }, [fetchData]);

    useEffect(() => {
        fetch(addr + `/api/confidential/static-station`)
            .then((response) => {
                if (response.ok) {
                    return response.json();
                } else {
                    setLoading(false);
                    throw new Error('Errore nella richiesta: ' + response.status);
                }
            })
            .then((data) => {
                setStaticStation(data);
            })
            .catch((error) => {
                console.error('Errore durante il fetch:', error);
            });
    }, [addr, setStaticStation, setLoading]);

    const groupedData = useMemo(() => {
        const hashmap = {};
        data.forEach(item => {
            const id = item.station_id || item.ID || item.station_ID || 'Sconosciuto';
            if (!hashmap[id]) hashmap[id] = [];
            hashmap[id].push(item);
        });
        return hashmap;
    }, [data]);

    const allAvailableIds = Object.keys(groupedData).sort();

    const toggleStationSelection = (id) => {
        setSelectedIds(prev =>
            prev.includes(id) ? prev.filter(i => i !== id) : [...prev, id]
        );
    };

    const formatNum = (val) => {
        const n = parseFloat(val);
        return isNaN(n) ? "--" : n.toFixed(1);
    };

    return (
        <div className="p-4 bg-gray-50 rounded-xl border border-gray-200 shadow-inner">
            {/* Toolbar Superiore */}
            <div className="flex justify-between items-center mb-6">
                <div className="flex items-center gap-2">
                    <MdSettingsInputAntenna className="text-blue-600 text-2xl" />
                    <h2 className="text-lg font-bold text-gray-800">Monitoraggio Selettivo</h2>
                </div>

                <div className="flex gap-2">
                    {/* Bottone per aprire il selettore */}
                    <button
                        onClick={() => setShowSelector(!showSelector)}
                        className={`flex items-center gap-2 px-4 py-2 rounded-lg border transition-all ${showSelector ? 'bg-blue-600 text-white border-blue-600' : 'bg-white text-gray-700 border-gray-300 hover:bg-gray-100'
                            }`}
                    >
                        <MdFilterList />
                        <span className="text-sm font-medium">Seleziona Stazioni ({selectedIds.length})</span>
                    </button>
                    <button onClick={fetchData} className="bg-gray-200 p-2 rounded-lg hover:bg-gray-300">
                        <MdRefresh size={20} />
                    </button>
                </div>
            </div>

            {/* Pannello di Selezione (Dropdown/Grid) */}
            {showSelector && (
                <div className="mb-6 p-4 bg-white border border-blue-200 rounded-lg shadow-sm animate-fadeIn">
                    <p className="text-[10px] font-bold text-blue-500 uppercase mb-3 tracking-widest">Scegli le stazioni da monitorare:</p>
                    <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 gap-2">
                        {allAvailableIds.map(id => (
                            <div
                                key={id}
                                onClick={() => toggleStationSelection(id)}
                                className={`flex items-center gap-2 p-2 rounded-md cursor-pointer border transition-all ${selectedIds.includes(id)
                                    ? 'bg-blue-50 border-blue-400 text-blue-700'
                                    : 'bg-gray-50 border-gray-200 text-gray-500 hover:border-gray-300'
                                    }`}
                            >
                                {selectedIds.includes(id) ? <MdCheckBox /> : <MdCheckBoxOutlineBlank />}
                                <span className="text-xs font-mono font-bold">{id}</span>
                            </div>
                        ))}
                    </div>
                    {selectedIds.length > 0 && (
                        <button
                            onClick={() => setSelectedIds([])}
                            className="mt-4 text-[10px] text-red-500 underline hover:text-red-700"
                        >
                            Deseleziona tutte
                        </button>
                    )}
                </div>
            )}

            {/* Visualizzazione Stazioni Selezionate */}
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                {selectedIds.length > 0 ? (
                    selectedIds.map(stationId => {
                        const measurements = groupedData[stationId] || [];
                        const lastM = measurements[0] || {}; // Ultima rilevazione

                        return (
                            <div key={stationId} className="bg-white border border-gray-200 rounded-xl p-4 shadow-sm hover:shadow-md transition-shadow">
                                <div className="flex justify-between items-start mb-4">
                                    <span className="bg-blue-600 text-white px-3 py-1 rounded-full text-[10px] font-black">
                                        STAZIONE {stationId}
                                    </span>
                                    <span className="text-[10px] text-gray-400 font-mono">
                                        LIVE: {lastM.orario ? new Date(lastM.orario).toLocaleTimeString() : '--'}
                                    </span>
                                </div>

                                <div className="grid grid-cols-3 gap-2">
                                    <div className="bg-orange-50 p-2 rounded-lg text-center">
                                        <p className="text-[9px] text-orange-400 font-bold uppercase">PM 2.5</p>
                                        <p className="text-lg font-black text-orange-600">{formatNum(lastM.pm2_5)}</p>
                                    </div>
                                    <div className="bg-red-50 p-2 rounded-lg text-center">
                                        <p className="text-[9px] text-red-400 font-bold uppercase">PM 10</p>
                                        <p className="text-lg font-black text-red-600">{formatNum(lastM.pm10)}</p>
                                    </div>
                                    <div className="bg-green-50 p-2 rounded-lg text-center">
                                        <p className="text-[9px] text-green-400 font-bold uppercase">Temp</p>
                                        <p className="text-lg font-black text-green-600">{formatNum(lastM.temperatura || lastM.temp)}°</p>
                                    </div>
                                </div>

                                <div className="mt-4 pt-3 border-t border-gray-50 flex justify-between items-center text-[10px] text-gray-400">
                                    <span>Storico: {measurements.length} campioni</span>
                                    <span>Sensore attivo</span>
                                </div>
                            </div>
                        );
                    })
                ) : (
                    <div className="col-span-full py-12 text-center border-2 border-dashed border-gray-200 rounded-xl">
                        <p className="text-gray-400 text-sm">
                            Nessuna stazione selezionata. <br />
                            Usa il tasto <strong className="text-blue-500">"Seleziona Stazioni"</strong> in alto per iniziare il monitoraggio.
                        </p>
                    </div>
                )}
            </div>
        </div>
    );
};

export default StaticStation;