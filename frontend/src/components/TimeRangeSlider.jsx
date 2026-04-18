import React from 'react';
import { Range, getTrackBackground } from 'react-range';

function TimeRangeSlider({ values, setValues, min, max }) {
  const STEP = 1000 * 60 * 1; // Step di 1 minuto

  function getAction(value) {
    console.log('Valore selezionato:', value);
    setValues(value);
  }

  const formatTime = (ms) => {
    return new Date(ms).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className="flex flex-col items-center w-full p-8 rounded-xl">
      <div className="flex justify-between w-full mb-6 text-black-500 font-mono text-sm">
        <span>Inizio: <b className="text-blue-400">{formatTime(values[0])}</b></span>
        <span>Fine: <b className="text-blue-400">{formatTime(values[1])}</b></span>
      </div>

      <Range
        values={values}
        step={STEP}
        min={min}
        max={max}
        onChange={(values) => getAction(values)}
        renderTrack={({ props, children }) => (
          <div
            onMouseDown={props.onMouseDown}
            onTouchStart={props.onTouchStart}
            className="h-9 flex w-full"
          >
            <div
              ref={props.ref}
              className="h-2 w-full rounded-full self-center"
              style={{
                background: getTrackBackground({
                  values,
                  colors: ['#334155', '#3b82f6', '#334155'],
                  min: min,
                  max: max
                })
              }}
            >
              {children}
            </div>
          </div>
        )}
        renderThumb={({ props, isDragged }) => (
          <div
            {...props}
            className={`h-5 w-5 rounded-full bg-white shadow-lg flex justify-center items-center outline-none ${
              isDragged ? 'ring-4 ring-blue-500/30' : 'ring-2 ring-blue-500'
            }`}
          >
            <div className={`h-2 w-1 bg-blue-500 rounded-full ${isDragged ? 'scale-125' : ''}`} />
          </div>
        )}
      />
      
      <div className="flex justify-between w-full mt-2 text-xs text-slate-500">
        <span>00:00</span>
        <span>06:00</span>
        <span>12:00</span>
        <span>18:00</span>
        <span>23:59</span>
      </div>
    </div>
  );
};

export default TimeRangeSlider;