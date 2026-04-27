import React, { useMemo } from 'react';
import { AreaChart, Area, XAxis, ResponsiveContainer, Tooltip } from 'recharts';

const DailyChartBar = ({ jsonData }) => {
  
  const chartData = useMemo(() => {
    if (!jsonData || jsonData.length === 0) return [];

    return [...jsonData].sort((a, b) => 
      new Date(a.orario).getTime() - new Date(b.orario).getTime()
    ).map(item => ({
      ...item,
      shortTime: new Date(item.orario).toLocaleTimeString('it-IT', { 
        hour: '2-digit', 
        minute: '2-digit',
        second: '2-digit' 
      })
    }));
  }, [jsonData]);

  return (
    <div className="w-full h-20 bg-white rounded-xl border border-gray-200 mt-4 shadow-sm overflow-hidden relative flex flex-col justify-center">
      {/* Intestazione sempre visibile */}
      <div className="absolute top-2 left-3 z-10 flex items-center gap-2">
        <span className="text-[10px] font-bold text-gray-400 uppercase tracking-widest">
          Serie Storica Campionamenti
        </span>
      </div>

      {chartData.length > 0 ? (
        <ResponsiveContainer width="100%" height="100%">
          <AreaChart data={chartData} margin={{ top: 35, right: 10, left: 10, bottom: 0 }}>
            <defs>
              <linearGradient id="colorPm" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="#3b82f6" stopOpacity={0.4}/>
                <stop offset="95%" stopColor="#3b82f6" stopOpacity={0}/>
              </linearGradient>
            </defs>
            <XAxis dataKey="shortTime" hide />
            <Tooltip 
              content={({ active, payload }) => {
                if (active && payload && payload.length) {
                  const data = payload[0].payload;
                  return (
                    <div className="bg-white/95 backdrop-blur-sm p-2 shadow-xl border border-blue-100 rounded-lg text-[10px]">
                      <p className="font-bold text-gray-500">{data.shortTime}</p>
                      <p className="text-blue-600 text-sm font-black">
                        {data.pm2_5} <span className="text-[9px] font-normal text-gray-400">µg/m³</span>
                      </p>
                    </div>
                  );
                }
                return null;
              }}
            />
            <Area 
              type="monotone" 
              dataKey="pm2_5" 
              stroke="#2563eb" 
              strokeWidth={2}
              fillOpacity={1} 
              fill="url(#colorPm)" 
              isAnimationActive={true}
            />
          </AreaChart>
        </ResponsiveContainer>
      ) : (
        /* Stato vuoto quando non ci sono dati */
        <div className="flex flex-col items-center justify-center pt-4">
          <p className="text-[10px] text-gray-300 italic font-medium uppercase tracking-tighter">
            Nessun dato disponibile per questa data
          </p>
          <div className="w-1/2 h-[1px] bg-gray-100 mt-1"></div>
        </div>
      )}
    </div>
  );
};

export default DailyChartBar;