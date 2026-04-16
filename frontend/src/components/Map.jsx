
import React from 'react';
import { MapContainer, TileLayer, CircleMarker, Tooltip } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet.heat';
import 'leaflet/dist/leaflet.css';

const getColor = (value) => {
  return value > 50 ? '#ef4444' :
         value > 30 ? '#f97316' :
         value > 15 ? '#eab308' :
                      '#22c55e';
};

function Map({ jsonData }) {
  const centroMappa = [40.774, 14.789];

  return (
    <div style={{ height: "100%", width: "100%" }}>
      <MapContainer center={centroMappa} zoom={15} style={{ height: "100%", width: "100%" }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        
        {jsonData.map((sensore, idx) => (
          <CircleMarker 
            key={idx}
            center={[sensore.lat, sensore.lon]}
            radius={10} // Dimensione fissa
            pathOptions={{ 
              fillColor: getColor(sensore.pm2_5), 
              fillOpacity: 0.8,
              color: 'white', // Bordo
              weight: 2
            }}
          >
            <Tooltip direction="top">
              <div>
                <strong>PM 2.5: {sensore.pm2_5} µg/m³</strong><br/>
                {sensore.orario}
              </div>
            </Tooltip>
          </CircleMarker>
        ))}
      </MapContainer>
    </div>
  );
}

export default Map;