import React from 'react';

function SideList({ jsonData, clickHandler }) {
  if (jsonData.length === 0) {
    return <p className="p-4 text-gray-500 text-sm">Nessun dato disponibile per questo intervallo.</p>;
  }

  console.log("FILTERED:");
  console.log(jsonData);

  return (

    <div className="side-list">
      <ul className='text-sm text-gray-700 p-2 space-y-2'>
        {jsonData.map((item, index) => (
          <li key={index}
            className='cursor-pointer p-3 border border-gray-200 rounded-lg bg-white hover:bg-blue-600 hover:text-white transition-all duration-200 shadow-sm'
            onClick={() => clickHandler(item.lat, item.lon)}>
            <div className="flex justify-between items-center mb-1">
              <span className="font-mono font-bold text-blue-500 group-hover:text-white">
                {item.orario.split(' ')[4] || item.orario.split(' ')[5]} 
              </span>
              <span className="text-[10px] uppercase opacity-60">Sats: {item.satelliti}</span>
            </div>
            <div className="grid grid-cols-2 gap-1 text-[11px]">
            <strong className='font-bold' > {item.orario.split(' ')[5]}</strong><br />
            <strong className='font-bold' >PM<sub>2.5</sub>:</strong> {item.pm2_5} µg/m³<br />
            <strong className='font-bold' >PM<sub>1.0</sub>:</strong> {item.pm1_0} µg/m³<br />
            <strong className='font-bold' >PM<sub>10</sub>:</strong> {item.pm10} µg/m³<br />
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default SideList;  