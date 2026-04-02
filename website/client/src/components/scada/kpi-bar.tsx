'use client';
import { Card, CardContent } from "@/components/ui/card";
import { Zap, Gauge, MapPin, Clock } from "lucide-react";

export function KpiBar({ telemetry, sessionData }: { telemetry: any; sessionData: any }) {
  // Safe extraction
  const voltage = telemetry?.voltage || 0.0;
  const vx = telemetry?.odom?.v_x || 0;
  const vy = telemetry?.odom?.v_y || 0;
  const speed = Math.sqrt(vx * vx + vy * vy);
  
  const distance = sessionData?.total_distance || 0;
  
  // Calculate uptime
  const startedAt = sessionData?.started_at ? new Date(sessionData.started_at) : null;
  const uptime = startedAt ? Math.floor((new Date().getTime() - startedAt.getTime()) / 1000) : 0;
  
  const hours = Math.floor(uptime / 3600);
  const minutes = Math.floor((uptime % 3600) / 60);
  const seconds = uptime % 60;
  const uptimeStr = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;

  const getVoltageColor = (v: number) => {
    if (v === 0) return 'text-muted-foreground';
    const pct = ((v - 21.0) / (25.2 - 21.0)) * 100;
    if (pct >= 30) return 'text-status-green';
    if (pct >= 15) return 'text-status-orange';
    return 'text-destructive';
  };
  
  const getBatteryPctStr = (v: number) => {
    if (v === 0) return '';
    const pct = Math.max(0, Math.min(100, ((v - 21.0) / (25.2 - 21.0)) * 100));
    return ` / ${Math.round(pct)}%`;
  };

  return (
    <div className="grid grid-cols-2 md:grid-cols-4 gap-4 w-full">
      <Card className="bg-card">
        <CardContent className="flex flex-col gap-1">
          <div className="text-[10px] uppercase font-bold tracking-widest text-muted-foreground flex items-center gap-2">
            <Zap size={12} /> 24V Battery
          </div>
          <div className={`text-2xl font-mono font-bold ${getVoltageColor(voltage)} tracking-tighter`}>
            {voltage > 0 ? `${voltage.toFixed(1)}V${getBatteryPctStr(voltage)}` : '--'}
          </div>
        </CardContent>
      </Card>

      <Card className="bg-card">
        <CardContent className="p-4 flex flex-col gap-1">
          <div className="text-[10px] uppercase font-bold tracking-widest text-muted-foreground flex items-center gap-2">
            <Gauge size={12} /> Speed
          </div>
          <div className="text-2xl font-mono font-bold text-foreground">
            {speed > 0 ? `${speed.toFixed(2)} m/s` : '0.00 m/s'}
          </div>
        </CardContent>
      </Card>

      <Card className="bg-card">
        <CardContent className="p-4 flex flex-col gap-1">
          <div className="text-[10px] uppercase font-bold tracking-widest text-muted-foreground flex items-center gap-2">
            <MapPin size={12} /> Distance (Session)
          </div>
          <div className="text-2xl font-mono font-bold text-status-blue">
            {distance.toFixed(1)}m
          </div>
        </CardContent>
      </Card>

      <Card className="bg-card">
        <CardContent className="p-4 flex flex-col gap-1">
          <div className="text-[10px] uppercase font-bold tracking-widest text-muted-foreground flex items-center gap-2">
            <Clock size={12} /> Uptime
          </div>
          <div className="text-2xl font-mono font-bold text-foreground">
            {uptimeStr}
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
