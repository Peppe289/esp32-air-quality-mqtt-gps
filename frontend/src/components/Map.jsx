
import React, { useEffect } from 'react';
import { MapContainer, TileLayer, CircleMarker, Tooltip, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet.heat';
import 'leaflet/dist/leaflet.css';

function ChangeView({ center, zoom }) {
  const map = useMap();
  useEffect(() => {
    map.flyTo(center, zoom, { duration: 0.4 });
  }, [center, zoom, map]);
  return null;
}

const getColor = (value) => {
  return value > 50 ? '#ef4444' :
    value > 30 ? '#f97316' :
      value > 15 ? '#eab308' :
        '#22c55e';
};

function Map({ jsonData, zoom, centroMappa }) {
  return (
    <div style={{ height: "100%", width: "100%" }}>
      <MapContainer center={centroMappa} zoom={zoom} style={{ height: "100%", width: "100%" }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />

        <ChangeView center={centroMappa} zoom={zoom} />

        {jsonData.map((sensore, idx) => (
          <CircleMarker
            key={idx}
            center={[sensore.lat, sensore.lon]}
            radius={10}
            pathOptions={{
              fillColor: getColor(sensore.pm2_5),
              fillOpacity: 0.8,
              color: 'white',
              weight: 2
            }}
          >
            <Tooltip direction="top">
              <div>
                <strong>PM 2.5: {sensore.pm2_5} µg/m³</strong><br />
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