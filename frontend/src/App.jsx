import React, { useEffect, useState } from 'react';
import Map from './components/Map';
import { ToastContainer, toast } from 'react-toastify';
import './style.css';

function App() {
  const [jsonData, setJsonData] = useState([]);
  const [errServer, setErrServer] = useState(false);
  const [updateInterval, setUpdateInterval] = useState(30000); // default 30s
  const [latency, setLatency] = useState(-1);

  const fetchData = React.useCallback(() => {
    const start = Date.now();
    fetch(`http://localhost:5000/api/data`)
      .then((response) => {
        setLatency(Date.now() - start);
        return response.json();
      })
      .then((data) => {
        setJsonData(data.results || []);
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
  }, [errServer, setJsonData, setErrServer]);
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
        <div className='flex space-x-4 items-end justify-end mt-2'>
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
        {/* <button onClick={fetchData} className='cursor-pointer bg-sky-500 text-white px-4 py-2 rounded m-2'>Ricarica</button> */}
      </div>
    </>
  );
}

export default App;