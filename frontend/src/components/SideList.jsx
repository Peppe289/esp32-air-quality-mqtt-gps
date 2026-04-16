import React from 'react';

function SideList({ jsonData, clickHandler }) {
  return (
    <div className="side-list">
      <ul className='text-sm text-gray-700 space-y-2 max-h-[80vh]'>
        {jsonData.map((item, index) => (
          <li key={index}
           className='cursor-pointer p-2 m-3 border rounded bg-gray-50 hover:bg-gray-100 transition-colors'
           onClick={() => clickHandler(item.lat, item.lon)}>
            <strong className='font-bold' >PM:</strong> {item.pm2_5} µg/m³<br/>
            <strong className='font-bold' >Orario:</strong> {item.orario}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default SideList;  