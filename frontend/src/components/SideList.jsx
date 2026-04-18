import React from 'react';

function SideList({ jsonData, clickHandler }) {
  return (
    <div className="side-list">
      <ul className='text-sm text-gray-700 space-y-2 max-h-[80vh]'>
        {jsonData.map((item, index) => (
          <li key={index}
            className='cursor-pointer p-2 m-3 border rounded bg-gray-50 hover:bg-blue-700 transition duration-200 hover:text-white'
            onClick={() => clickHandler(item.lat, item.lon)}>
            <strong className='font-bold' > {item.orario.split(' ')[5]}</strong><br />
            <strong className='font-bold' >PM<sub>2.5</sub>:</strong> {item.pm2_5} µg/m³<br />
            <strong className='font-bold' >PM<sub>1.0</sub>:</strong> {item.pm1_0} µg/m³<br />
            <strong className='font-bold' >PM<sub>10</sub>:</strong> {item.pm10} µg/m³<br />
            <strong className='font-bold' >#satelliti:</strong> {item.satelliti}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default SideList;  