
import React, { useEffect, useState } from 'react';
import { useMapEvents, MapContainer, TileLayer, CircleMarker, Polygon, Tooltip, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet.heat';
import 'leaflet/dist/leaflet.css';

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

function StaticStationSelection({staticStation, setSelectedStation}) {
    useMapEvents({
    click(e) {
      const { lat, lng } = e.latlng;
      staticStation.forEach((element) => {
        if (formatCoords(element.lat, true) === formatCoords(lat, true)
            && formatCoords(element.lon, false) === formatCoords(lng, false)) {
          setSelectedStation(prev =>
            prev.includes(element.ID) ? prev.filter(i => i !== element.ID) : [...prev, element.ID]
        );
        }
      });
    },
  });

  return null;
}

function AreaSelector({ points, setPoints, staticStation }) {
  useMapEvents({
    click(e) {
      const { lat, lng } = e.latlng;
      let isValid = true;
      if (staticStation) {
        staticStation.forEach(element => {
          if (formatCoords(element.lat, true) === formatCoords(lat, true)
            && formatCoords(element.lon, false) === formatCoords(lng, false)) {
            isValid = false;
          }
        });
      }

      if (isValid) setPoints((prev) => [...prev, [lat, lng]]);
    },
  });

  return (
    <>
      {/* Punti piccolini (CircleMarker non sgranano con lo zoom) */}
      {points.map((p, i) => (
        <CircleMarker
          key={i}
          center={p}
          radius={4}
          pathOptions={{ color: 'red', fillColor: 'red', fillOpacity: 1 }}
        />
      ))}

      {/* Disegna il poligono se ci sono almeno 3 punti */}
      {points.length >= 3 && (
        <Polygon
          positions={points}
          pathOptions={{ color: 'blue', fillColor: 'blue', fillOpacity: 0.2 }}
        />
      )}
    </>
  );
}

function StaticMarkers({ staticStation, selectedPoint }) {
  const circleColor = ['#64748b', '#155dfc'];

  const setMarker = (punto, selected) => {
    return (<CircleMarker
      key={punto.id}
      center={[punto.lat, punto.lon]}
      radius={6}
      pathOptions={{
        fillColor: circleColor[selected ? 1 : 0],
        color: '#334155',
        weight: 1,
        fillOpacity: 0.8
      }}
    >
      <Tooltip direction="top" offset={[0, -5]}>
        <div className="text-xs font-sans">
          <div className="font-bold border-b border-gray-200 mb-1">ID: {punto.ID}</div>
          <div>Lat: {formatCoords(punto.lat, true)}</div>
          <div>Lon: {formatCoords(punto.lon, false)}</div>
        </div>
      </Tooltip>
    </CircleMarker>)
  }

  const isSelected = (punto) => {
    let selected = false;
      if (selectedPoint) {
        selectedPoint.forEach(element => {
          if (element === punto.ID) {
            selected = true;
          }
        });
      }
    return selected;
  }

  return (
    <>
      {staticStation.map((punto) => (
        setMarker(punto, isSelected(punto))
      ))}
    </>
  );
}

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

function Map({ jsonData, zoom, centroMappa, staticStation, userPoints, setUserPoints, selectedPoint, setSelectedStation }) {

  return (
    <div style={{ height: "100%", width: "100%" }}>
      <MapContainer center={centroMappa} zoom={zoom} style={{ height: "100%", width: "100%" }} zoomControl={false}>
        <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
        <AreaSelector points={userPoints} setPoints={setUserPoints} staticStation={staticStation}></AreaSelector>

        <ChangeView center={centroMappa} zoom={zoom} />
        <StaticStationSelection staticStation={staticStation} setSelectedStation={setSelectedStation}/>

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

        <StaticMarkers staticStation={staticStation} selectedPoint={selectedPoint} />
      </MapContainer>
    </div>
  );
}

export default Map;