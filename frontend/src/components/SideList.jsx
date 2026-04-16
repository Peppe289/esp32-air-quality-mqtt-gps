import React from 'react';

function SideList({ jsonData }) {
  return (
    <div className="side-list">
      <h2>Dati Sensori</h2>
      <ul>
        {jsonData.map((item, index) => (
          <li key={index}>
            <strong>PM:</strong> {item.pm2_5} µg/m³<br/>
            <strong>Orario:</strong> {item.orario}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default SideList;  