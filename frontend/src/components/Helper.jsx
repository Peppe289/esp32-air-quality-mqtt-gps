//import React, { useState, useEffect, useCallback } from 'react';
import { toast } from 'react-toastify';
import TimeRangeSlider from './TimeRangeSlider';
import { MdOutlineRemoveCircle } from 'react-icons/md';

const formatCoords = (val, isLat) => {
  const absolute = Math.abs(val);
  const degrees = Math.floor(absolute);
  const minutesNotTruncated = (absolute - degrees) * 60;
  const minutes = Math.floor(minutesNotTruncated);
  const seconds = Math.floor((minutesNotTruncated - minutes) * 60);

  const cardinal = isLat
    ? (val >= 0 ? 'N' : 'S')
    : (val >= 0 ? 'E' : 'W');

  return `${degrees}° ${minutes}' ${seconds}" ${cardinal}`;
};

function Helper({ dayDate, handleDateChange, updateInterval, setUpdateInterval, latency, userPoints, setUserPoints, stats }) {
  return (
    <>
      {/* Lista dei punti creati sulla mappa */}
      <div className='flex flex-col items-center justify-between gap-4 mt-4 p-4 bg-gray-50 rounded-xl border border-gray-200'>
        <p>Area selezionata</p>
        {/* Lista dei punti settati */}
        <div className='overflow-y-scroll h-50 bg-white w-[95%]'>
          {userPoints.length > 0 ? <ul>
            {userPoints.map((item, index) => (
              <div className='flex flex-col justify-center items-center text-sm'>
                <div className='flex flex-row items-center w-full justify-center gap-4'>
                  <div className='bg-blue-700 p-1.5 rounded-4xl text-white w-9 h-9 items-baseline justify-center text-center'>
                    <p>{index}</p>
                  </div>
                  <li key={index}>
                    Langitudine: {formatCoords(item[0], true)}<br />
                    Longitudine: {formatCoords(item[1], false)}
                  </li>
                </div>
                {index < userPoints.length - 1 && (
                  <span className='border-b border-gray-200 w-[70%] my-1'></span>
                )}
              </div>
            ))}
          </ul> : <div className='items-baseline justify-center text-center'> <p className='w-[80%] m-auto'>Premi sulla mappa per settare dei punti</p> </div>}
        </div>

        <div>
          <button style={{
            padding: '5px 10px', cursor: 'pointer',
            backgroundColor: '#ff3300', fontSize: '15px',
            color: 'white', border: 'none', borderRadius: '6px'
          }} onClick={() => { setUserPoints([]) }}>Elimina</button>
        </div>

        <div className="bg-blue-50 p-4 rounded-lg mt-0 border border-blue-200">
            <h3 className="text-blue-800 font-bold text-sm mb-2 uppercase">Analisi Area</h3>
            <div className="grid grid-cols-1 gap-2 text-sm">
              <p><strong>Campioni trovati:</strong> {stats.count}</p>
              <p><strong>Media PM 1.0</strong> {stats.avgPm1} µg/m<sup>3</sup></p>
              <p><strong>Media PM 2.5:</strong> {stats.avgPm25} µg/m<sup>3</sup></p>
              <p><strong>Media PM 10:</strong> {stats.avgPm10} µg/m<sup>3</sup></p>
            </div>
          </div>
      </div>

      {/* Controlli Inferiori */}
      <div className="flex flex-col items-center justify-between gap-4 mt-4 p-4 bg-gray-50 rounded-xl border border-gray-200">

        {/* Selezione Data */}
        <div className="w-full flex items-center justify-between gap-3">
          <label htmlFor="dayDate" className="text-sm font-medium text-gray-700">Giorno:</label>
          <input
            type="date"
            id="dayDate"
            className="border border-gray-300 rounded-md px-3 py-1.5 text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500 outline-none transition-all"
            value={dayDate}
            onChange={handleDateChange}
          />
        </div>

        {/* Info Intervallo e Latenza */}
        <div className="w-full flex flex-col items-center justify-between">
          <div className="flex w-full items-center justify-between">
            <label htmlFor="updateInterval" className="text-xs font-semibold text-gray-500 uppercase">Polling (ms):</label>
            <input
              readOnly={dayDate !== (new Date().toISOString().split('T')[0])}
              type="number"
              id="updateInterval"
              className={`border rounded md px-2 py-1 w-20 text-right text-sm font-mono ${dayDate !== (new Date().toISOString().split('T')[0])
                ? 'bg-gray-100 text-gray-400 cursor-not-allowed'
                : 'border-gray-300 focus:ring-1 focus:ring-blue-400'
                }`}
              onClick={dayDate !== (new Date().toISOString().split('T')[0]) ? () => toast.error('Seleziona la data odierna per modificare il polling!') : undefined}
              value={updateInterval}
              onChange={(e) => setUpdateInterval(Number(e.target.value))}
            />
          </div>
          <div className="flex items-center gap-1 mt-1">
            <span className={`w-2 h-2 rounded-full ${latency < 200 ? 'bg-green-500' : 'bg-yellow-500'}`}></span>
            <p className="text-xs text-gray-500 font-medium">Latency: {latency} ms</p>
          </div>
        </div>
      </div>
    </>
  );
};

export default Helper;