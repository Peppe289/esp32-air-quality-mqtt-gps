import React, { useEffect, useState, useCallback, useMemo } from 'react';
import Map from './components/Map';
import { Slide, ToastContainer, toast } from 'react-toastify';
import TimeRangeSlider from './components/TimeRangeSlider';
import './style.css';
import SideList from './components/SideList';
import { MdEdit } from "react-icons/md";
import StaticStation from './components/StaticStation';

function getWindowDimensions() {
  const { innerWidth: width, innerHeight: height } = window;
  console.log(`Current width: ${width}\nCurrent height: ${height}`);
  return {
    width,
    height
  };
}

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
  const [windowDimensions, setWindowDimensions] = useState(getWindowDimensions());

  useEffect(() => {
    function handleResize() {
      setWindowDimensions(getWindowDimensions());
    }

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);


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
        console.log('Giorno: ', dayDate);
        setJsonData(data.results);
        if (errServer) toast.success('Dati filtrati ricevuti con successo!');
        setErrServer(false);
      })
      .catch((error) => {
        //if (!errServer) {
        //  toast.error('Errore durante il filtraggio dei dati!');
        //}
        console.error('Error fetching filtered data:', error);
        setErrServer(true);
      });
  }, [dayDate, addr, route, errServer]);

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

  return (
    <>
      <div className='flex items-center justify-center flex-col mt-4'>
        <h1 className='text-3xl font-bold text-gray-800'>PM 2.5 Monitoraggio</h1>
        <p className='text-sm text-gray-500'>Visualizza i dati dei sensori di PM 2.5 in tempo reale</p>
      </div>
      <div className='m-3'>
        <div className="flex w-full h-[100vh] lg:h-[80vh] border-2 border-gray-300 rounded overflow-hidden p-4">
          {windowDimensions.width > 1024 &&
            <div className="w-1/5 overflow-y-auto h-full border-r border-gray-300 bg-white">
              <h2 className='text-lg font-bold mb-2 sticky top-0 bg-white' >Dati Sensori</h2>
              <SideList jsonData={filteredData} clickHandler={sideMarkerClickHandler} />
            </div>
          }
          <div className="flex-1 h-full">
            <Map centroMappa={centroMappa} jsonData={filteredData} zoom={zoom} staticStation={staticStation} />
          </div>
        </div>
        <ToastContainer />
        <div className='flex flex-col lg:flex-row lg:space-x-4 lg:justify-between mt-2'>
          <div className='flex flex-row items-center space-x-2'>
            <label htmlFor="dayDate">Seleziona Giorno:</label>
            <input
              className='border-2 border-gray-300 rounded m-2 p-1 w-48'
              type="date"
              id="dayDate"
              value={dayDate}
              onChange={handleDateChange}
            />
          </div>
          <TimeRangeSlider min={MIN} max={MAX} values={timelineRange} setValues={setTimelineRange} />
          <div className='flex flex-col items-end'>
            <label htmlFor="updateInterval">Update Interval (ms):</label>
            <input readOnly={dayDate == (new Date().toISOString().split('T')[0]) ? false : true}
              className={`border-2 border-gray-300 rounded m-2 p-1 w-24 ${dayDate == (new Date().toISOString().split('T')[0]) ? '' : 'bg-gray-200 cursor-pointer'}`}
              type="number"
              id="updateInterval"
              onClick={dayDate != (new Date().toISOString().split('T')[0]) ? () => toast.error('Per modificare l\'intervallo di aggiornamento, seleziona la data odierna!') : undefined}
              value={updateInterval}
              onChange={(e) => setUpdateInterval(Number(e.target.value))}
            />
            <p className='text-sm text-gray-500'>Latency: {latency} ms</p>
          </div>
        </div>
      </div>
      {windowDimensions.width <= 1024 &&
          <div className="w-full overflow-y-auto h-full border-r border-gray-300 bg-white">
            <h2 className='text-lg font-bold mb-2 sticky top-0 bg-white' >Dati Sensori</h2>
            <SideList jsonData={filteredData} clickHandler={sideMarkerClickHandler} />
          </div>
        }
      {isOnVPN && <StaticStation setStaticStation={setStaticStation} loading={isOnVPN} setLoading={setIsOnVPN} />}
    </>
  );
}

export default App;