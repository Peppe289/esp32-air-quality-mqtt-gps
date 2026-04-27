import React from 'react';
import { Range, getTrackBackground } from 'react-range';

function TimeRangeSlider({ values, setValues, min, max }) {
  const STEP = 1000 * 60 * 1; // 1 minuto

  // Evitiamo errori se min e max sono uguali (es. un solo dato)
  const safeMax = max <= min ? min + STEP : max;

  const formatTime = (ms) => {
    return new Date(ms).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className="flex flex-col items-center w-full p-8 rounded-xl bg-white border border-gray-100 shadow-sm">
      <div className="flex justify-between w-full mb-6 text-slate-600 font-mono text-sm">
        <div className="flex flex-col">
          <span className="text-[10px] text-slate-400 uppercase">Da</span>
          <b className="text-blue-500">{formatTime(values[0])}</b>
        </div>
        <div className="flex flex-col items-end">
          <span className="text-[10px] text-slate-400 uppercase">A</span>
          <b className="text-blue-500">{formatTime(values[1])}</b>
        </div>
      </div>

      <Range
        values={values}
        step={STEP}
        min={min}
        max={safeMax}
        onChange={(v) => setValues(v)}
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
                  colors: ['#e2e8f0', '#3b82f6', '#e2e8f0'],
                  min: min,
                  max: safeMax
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
            className={`h-5 w-5 rounded-full bg-white shadow-lg flex justify-center items-center outline-none border-2 border-blue-500`}
          >
            <div className={`h-2 w-1 bg-blue-500 rounded-full ${isDragged ? 'scale-125' : ''}`} />
          </div>
        )}
      />
      
      {/* Etichette dinamiche basate sui dati reali */}
      <div className="flex justify-between w-full mt-2 text-[10px] font-bold text-slate-400 uppercase tracking-tighter">
        <span>Inizio Dati ({formatTime(min)})</span>
        <span>Fine Dati ({formatTime(max)})</span>
      </div>
    </div>
  );
};

export default TimeRangeSlider;