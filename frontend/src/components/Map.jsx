
import React, { useState } from 'react';
import { MapContainer, TileLayer, useMap, CircleMarker, Tooltip } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet.heat';
import 'leaflet/dist/leaflet.css';

function HeatmapLayer({ points }) {
  const map = useMap();
  React.useEffect(() => {
    if (!map || !points) return;
    const heatLayer = L.heatLayer(points, { radius: 20, blur: 20 });
    heatLayer.addTo(map);
    return () => map.removeLayer(heatLayer);
  }, [map, points]);
  return null;
}

function Map({ jsonData }) {
  //const [jsonData, setjsonData] = useState([
  //  { lat: 40.7744, lon: 14.7891, pm: 45, orario: "10:30", intensita: 0.8 },
  //  { lat: 40.7710, lon: 14.7920, pm: 12, orario: "10:35", intensita: 0.3 },
  //  { lat: 40.7710, lon: 14.7921, pm: 85, orario: "10:40", intensita: 1.0 },
  //  { lat: 40.7710, lon: 14.7922, pm: 85, orario: "10:40", intensita: 0.8 },
  //  { lat: 40.7710, lon: 14.7923, pm: 85, orario: "10:40", intensita: 0.9 },
  //]);

  const centroMappa = [40.774, 14.789];

  const heatmapPoints = jsonData.map(d => [d.lat, d.lon, d.pm2_5 > 40 ? 1.0 : d.pm2_5 / 40]); // Normalizza PM2.5 a un range 0-1

  return (
    <div style={{ height: "100%", width: "100%" }}>
      <MapContainer center={centroMappa} zoom={15} style={{ height: "100%", width: "100%" }}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        
        {/* 1. Lo strato visivo Heatmap */}
        <HeatmapLayer points={heatmapPoints} />

        {/* 2. Lo strato interattivo (Marker Invisibili) */}
        {jsonData.map((sensore, idx) => (
          <CircleMarker 
            key={idx}
            center={[sensore.lat, sensore.lon]}
            radius={8}
            pathOptions={{ 
              fillColor: 'transparent', 
              color: 'transparent',
              stroke: false 
            }}
          >
            {/* Tooltip che appare all'hover */}
            <Tooltip direction="top" permanent={false} offset={[0, -10]} opacity={1}>
              <div style={{ fontSize: '14px' }}>
                <strong>Dati Sensore</strong><br/>
                PM: {sensore.pm2_5} µg/m³<br/>
                Orario: {sensore.orario}
              </div>
            </Tooltip>
          </CircleMarker>
        ))}
      </MapContainer>
    </div>
  );
}

export default Map;