import React, { useEffect, useState } from 'react';
import Map from './components/Map';
import { ToastContainer, toast } from 'react-toastify';
import TimeRangeSlider from './components/TimeRangeSlider';
import './style.css';
import { MdEdit } from "react-icons/md";

function App() {
  const [ipAddress, setIpAddress] = useState("http://localhost:5000");
  const [jsonData, setJsonData] = useState([]);
  const [errServer, setErrServer] = useState(false);
  const [updateInterval, setUpdateInterval] = useState(30000); // default 30s
  const [latency, setLatency] = useState(-1);
  const [dayDate, setDayDate] = useState(new Date().toISOString().split('T')[0]); // default oggi
  // const [timelineRange, setTimelineRange] = useState([0, 100]); // Default range
  const [editAddr, setEditAddr] = useState(false);

  const validateIpAddress = (ip) => {
    const regex = /^(https?:\/\/)?((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)(:\d+)?$/;
    if (!regex.test(ip)) {
      toast.error('Indirizzo IP non valido!');
      return;
    }

    fetch(`${ip}/api/data`)
      .then((response) => {
        if (response.ok) {
          setIpAddress(ip);
          toast.success('Indirizzo IP aggiornato con successo!');
          setErrServer(false);
        }
      })
  }

  const formatReadableDate = (isoDate) => {
    const options = { year: 'numeric', month: 'long', day: 'numeric', hour: '2-digit', minute: '2-digit', second: '2-digit' };
    return new Date(isoDate).toLocaleDateString('it-IT', options);
  };

  const fetchData = React.useCallback(() => {
    const start = Date.now();
    fetch(`${ipAddress}/api/data`)
      .then((response) => {
        setLatency(Date.now() - start);
        return response.json();
      })
      .then((data) => {
        const formattedData = data.results.map((item) => ({
          ...item,
          orario: formatReadableDate(item.orario),
        }));
        setJsonData(formattedData);
        if (errServer) {
          toast.success('Connessione al server ristabilita!');
          setErrServer(false);
        }
      })
      .catch((error) => {
        if (!errServer) {
          toast.error('Errore di connessione al server!');
          setErrServer(true);
        }
        console.error('Error fetching data:', error);
      });
  }, [errServer, setJsonData, setErrServer, ipAddress]);

  const fetchFilteredData = () => {
    if (!dayDate) {
      toast.error('Inserisci la data!');
      return;
    }

    fetch(`${ipAddress}/api/data?day=${dayDate}`)
      .then((response) => response.json())
      .then((data) => {
        const formattedData = data.results.map((item) => ({
          ...item,
          orario: formatReadableDate(item.orario),
        }));
        setJsonData(formattedData);
        toast.success('Dati filtrati ricevuti con successo!');
      })
      .catch((error) => {
        toast.error('Errore durante il filtraggio dei dati!');
        console.error('Error fetching filtered data:', error);
      });
  };

  // const handleTimelineChange = (e) => {
  //   const value = Number(e.target.value);
  //   const newRange = [...timelineRange];
  //   if (e.target.id === 'timelineStart') {
  //     newRange[0] = value;
  //   } else {
  //     newRange[1] = value;
  //   }
  //   setTimelineRange(newRange);
  // };

  // const fetchTimelineData = () => {
  //   const [start, end] = timelineRange;
  //   fetch(`http://localhost:5000/api/data?timelineStart=${start}&timelineEnd=${end}`)
  //     .then((response) => response.json())
  //     .then((data) => {
  //       setJsonData(data.results || []);
  //       toast.success('Dati aggiornati in base alla timeline!');
  //     })
  //     .catch((error) => {
  //       toast.error('Errore durante l\'aggiornamento dei dati dalla timeline!');
  //       console.error('Error fetching timeline data:', error);
  //     });
  // };

  useEffect(() => {
    fetchData();
    const interval = setInterval(fetchData, updateInterval);
    return () => clearInterval(interval);
  }, [errServer, fetchData, updateInterval]);

  return (
    <>
      <div className='m-3'>
        <ToastContainer />
        <div style={{ height: "80vh" }} className='border-2 border-gray-300 rounded'>
          <Map jsonData={jsonData} />
        </div>
        <div className='flex space-x-4 justify-between mt-2'>
          <div className='flex flex-row items-center space-x-2'>
            <label htmlFor="dayDate">Seleziona Giorno:</label>
            <input
              className='border-2 border-gray-300 rounded m-2 p-1 w-48'
              type="date"
              id="dayDate"
              value={dayDate}
              onChange={(e) => {
                setDayDate(e.target.value);
                fetchFilteredData();
              }}
            />
          </div>
          <TimeRangeSlider />
          <div className='flex flex-col items-end'>
            <label htmlFor="updateInterval">Update Interval (ms):</label>
            <input className='border-2 border-gray-300 rounded m-2 p-1 w-24'
              type="number"
              id="updateInterval"
              value={updateInterval}
              onChange={(e) => setUpdateInterval(Number(e.target.value))}
            />
            <p className='text-sm text-gray-500'>Latency: {latency} ms</p>
          </div>
        </div>
      </div>
      <div id='current-server' className='flex item-center justify-center flex-row bottom-2 right-2 text-xs text-gray-500'>
        <label htmlFor="ipAddress" >Server IP:
          <input type="text" className={`m-3 ${!editAddr && `bg-gray-200`}`} id="ipAddress" value={ipAddress}
            readOnly={!editAddr} onChange={(e) => validateIpAddress(e.target.value)} />
        </label>
        <p className='m-3 cursor-pointer' onClick={() => setEditAddr(!editAddr)}><MdEdit /></p>
      </div>
    </>
  );
}

export default App;