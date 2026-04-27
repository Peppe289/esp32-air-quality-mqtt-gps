import React, { useEffect, useState, useCallback, useMemo } from 'react';
import { MdMenu, MdClose } from "react-icons/md";
import Map from './components/Map';
import { Slide, ToastContainer, toast } from 'react-toastify';
import TimeRangeSlider from './components/TimeRangeSlider';
import './style.css';
import SideList from './components/SideList';
import { MdEdit } from "react-icons/md";
import StaticStation from './components/StaticStation';
import Helper from './components/Helper';
import DailyChartBar from './components/DailyChartBar';

function App() {
  const [jsonData, setJsonData] = useState([]);
  const [errServer, setErrServer] = useState(false);
  const [updateInterval, setUpdateInterval] = useState(30000); // default 30s
  const [latency, setLatency] = useState(-1);
  const [dayDate, setDayDate] = useState(new Date().toISOString().split('T')[0]); // default oggi
  const MIN = useMemo(() => new Date(dayDate + 'T00:00:00').getTime(), [dayDate]);
  const MAX = useMemo(() => new Date(dayDate + 'T23:59:59').getTime(), [dayDate]);
  const [timelineRange, setTimelineRange] = useState([MIN, MAX]); // Default range
  const [centroMappa, setCentroMappa] = useState([40.774, 14.789]);
  const [zoom, setZoom] = useState(15);
  const [staticStation, setStaticStation] = useState([]);
  const [isOnVPN, setIsOnVPN] = useState(true);
  const addr = import.meta.env.VITE_SERVER_ADDR || "";
  const [route, setRoute] = useState(addr + '/api/confidential');
  const [isSidebarOpen, setIsSidebarOpen] = useState(false);
  const [isHelperOpen, setIsHelperOpen] = useState(false);
  const [userPoints, setUserPoints] = useState([]);

  const sideMarkerClickHandler = (lat, lon) => {
    setCentroMappa([lat, lon]);
    setZoom(18); // Zoom più ravvicinato
    //console.log(`Centro mappa aggiornato a: ${lat}, ${lon}`);
  }

  const handleDateChange = (e) => {
    const newDate = e.target.value;
    const newMin = new Date(newDate + 'T00:00:00').getTime();
    const newMax = new Date(newDate + 'T23:59:59').getTime();

    setDayDate(newDate);
    setTimelineRange([newMin, newMax]);
  };

  const formatReadableDate = (isoDate) => {
    const options = { year: 'numeric', month: 'long', day: 'numeric', hour: '2-digit', minute: '2-digit', second: '2-digit' };
    return new Date(isoDate).toLocaleDateString('it-IT', options);
  };

  const fetchFilteredData = useCallback(() => {
    if (!dayDate) {
      toast.error('Inserisci la data!');
      return;
    }

    const start = new Date();

    fetch(`${route}/data?day=${dayDate}`)
      .then((response) => {
        if (!response.ok) {
          setRoute(addr + '/api/public');
        }
        return response.json();
      })
      .then((data) => {
        const end = new Date();
        setLatency(end - start);

        const results = data.results;
        setJsonData(results); // 1. Aggiorna i dati

        // 2. Calcola i limiti e aggiorna lo slider QUI dentro
        if (results && results.length > 0) {
          const timestamps = results.map(d => new Date(d.orario).getTime());
          const dMin = Math.min(...timestamps);
          const dMax = Math.max(...timestamps);
          setTimelineRange([dMin, dMax]); // Aggiornamento "batch" (nello stesso ciclo)
        } else {
          // Se non ci sono dati, resetta al giorno intero
          setTimelineRange([MIN, MAX]);
        }

        if (errServer) toast.success('Dati filtrati ricevuti con successo!');
        setErrServer(false);
      })
      .catch((error) => {
        console.error('Error fetching filtered data:', error);
        setErrServer(true);
      });
  }, [dayDate, addr, route, errServer, MIN, MAX]); // Aggiungi MIN e MAX alle dipendenze

  useEffect(() => {
    fetchFilteredData();
    const interval = setInterval(fetchFilteredData, updateInterval);
    return () => clearInterval(interval);
  }, [fetchFilteredData, updateInterval]);

  const filteredData = useMemo(() => {
    return jsonData
      .filter((item) => {
        const itemTime = new Date(item.orario).getTime();
        return itemTime >= timelineRange[0] && itemTime <= timelineRange[1];
      })
      .map((item) => ({
        ...item,
        // Nota: formattiamo la data solo per la visualizzazione nei popup della mappa
        orario: formatReadableDate(item.orario),
      }));
  }, [jsonData, timelineRange]);

  const isInside = (point, vs) => {
    // point è [lat, lng], vs è l'array di vertici [[lat, lng], ...]
    const x = point[0], y = point[1];
    let inside = false;
    for (let i = 0, j = vs.length - 1; i < vs.length; j = i++) {
      const xi = vs[i][0], yi = vs[i][1];
      const xj = vs[j][0], yj = vs[j][1];
      const intersect = ((yi > y) !== (yj > y)) &&
        (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  };

  const dataInArea = useMemo(() => {
    // Se non hai almeno 3 punti, il poligono non esiste.
    // Quindi usa tutti i dati
    if (userPoints.length < 3) return filteredData;

    // Filtriamo jsonData (assumendo che ogni oggetto abbia .lat e .lon)
    return filteredData.filter(sensor =>
      isInside([sensor.lat, sensor.lon], userPoints)
    );
  }, [userPoints, filteredData]);

  const stats = useMemo(() => {
    if (dataInArea.length === 0) return {
      avgPm25: 'N/A',
      avgPm10: 'N/A',
      avgPm1: 'N/A',
      count: 0
    };

    const total = dataInArea.reduce((acc, curr) => {
      return {
        pm25: acc.pm25 + curr.pm2_5,
        pm10: acc.pm10 + curr.pm10,
        pm1: acc.pm1 + curr.pm1_0
      };
    }, { pm25: 0, pm10: 0, pm1: 0 });

    return {
      avgPm25: (total.pm25 / dataInArea.length).toFixed(2),
      avgPm10: (total.pm10 / dataInArea.length).toFixed(2),
      avgPm1: (total.pm1 / dataInArea.length).toFixed(2),
      count: dataInArea.length
    };
  }, [dataInArea]);

  return (
    <>
      <div className='flex items-center justify-center flex-col mt-4'>
        <h1 className='text-3xl font-bold text-gray-800'>PM 2.5 Monitoraggio</h1>
        <p className='text-sm text-gray-500'>Visualizza i dati dei sensori di PM 2.5 in tempo reale</p>
      </div>
      <div className="m-auto w-[96%] flex justify-center items-center">
        <div className="m-4 relative flex w-full h-[70vh] border border-gray-300 rounded-xl shadow-inner overflow-hidden">

          {/* Pulsante Toggle Sinistro (SideList) */}
          <button
            onClick={() => setIsSidebarOpen(!isSidebarOpen)}
            className="absolute top-4 left-4 z-[1100] p-2 bg-white rounded-md shadow-md border border-gray-200 hover:bg-gray-100 transition-colors"
            title={isSidebarOpen ? "Chiudi lista" : "Apri lista"}
          >
            {isSidebarOpen ? <MdClose size={24} /> : <MdMenu size={24} />}
          </button>

          {/* Pulsante Toggle Destro (Helper) - NUOVO */}
          <button
            onClick={() => setIsHelperOpen(!isHelperOpen)}
            className="absolute top-4 right-4 z-[1100] p-2 bg-white rounded-md shadow-md border border-gray-200 hover:bg-gray-100 transition-colors"
            title={isHelperOpen ? "Chiudi Helper" : "Apri Helper"}
          >
            {isHelperOpen ? <MdClose size={24} /> : <MdEdit size={24} />}
          </button>

          {/* SideList (Lato Sinistro) */}
          <div className={`absolute top-4 left-4 z-[1000] w-72 max-h-[calc(100%-2rem)] overflow-y-auto bg-white/90 backdrop-blur-sm shadow-lg rounded-lg border border-gray-200 transition-all duration-300 ease-in-out ${isSidebarOpen ? 'translate-x-0 opacity-100' : '-translate-x-full opacity-0 pointer-events-none'
            }`}>
            <div className="p-3 pt-14 border-b border-gray-100 bg-gray-50/50 sticky top-0 z-10">
              <h2 className="text-sm font-bold text-gray-700 uppercase tracking-wider">Dati Sensori</h2>
            </div>
            <SideList jsonData={filteredData} clickHandler={sideMarkerClickHandler} />
          </div>

          {/* Helper (Lato Destro) - NUOVO */}
          <div className={`absolute top-4 right-4 z-[1000] w-80 max-h-[calc(100%-2rem)] overflow-y-auto bg-white/95 backdrop-blur-sm shadow-lg rounded-lg border border-gray-200 transition-all duration-300 ease-in-out ${isHelperOpen ? 'translate-x-0 opacity-100' : 'translate-x-full opacity-0 pointer-events-none'
            }`}>
            <div className="p-3 pt-14 border-b border-gray-100 bg-gray-50/50 sticky top-0 z-10">
              <h2 className="text-sm font-bold text-gray-700 uppercase tracking-wider">Strumenti Analisi</h2>
            </div>
            <div className="p-4">
              <Helper
                userPoints={userPoints}
                setUserPoints={setUserPoints}
                dayDate={dayDate}
                handleDateChange={handleDateChange}
                updateInterval={updateInterval}
                setUpdateInterval={setUpdateInterval}
                latency={latency}
                stats={stats}
              />
            </div>
          </div>

          {/* Mappa */}
          <div className="flex-1 h-full z-10">
            <Map userPoints={userPoints} setUserPoints={setUserPoints} centroMappa={centroMappa} jsonData={filteredData} zoom={zoom} staticStation={staticStation} />
          </div>
        </div>
      </div>

      {/* Inseriscilo tra la mappa e lo slider */}
      <div className="w-11/12 m-auto">
        <DailyChartBar jsonData={filteredData} /> {/* Usiamo jsonData (tutto il giorno) non filteredData */}

        <div className="mt-2">
          <TimeRangeSlider min={MIN} max={MAX} values={timelineRange} setValues={setTimelineRange} />
        </div>
      </div>
      {isOnVPN && <StaticStation setStaticStation={setStaticStation} loading={isOnVPN} setLoading={setIsOnVPN} />}

      <ToastContainer transition={Slide} />
    </>
  );
}

export default App;