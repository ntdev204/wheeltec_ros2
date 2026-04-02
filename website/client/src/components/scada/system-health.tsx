'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { Server, Zap, Cpu } from 'lucide-react';
import { Badge } from '@/components/ui/badge';

export function SystemHealth() {
  const { isConnected, telemetry } = useRobotState();

  const voltage = telemetry?.voltage ?? 0.0;
  
  // Calculate battery percentage for a 24V battery (6S LiPo: roughly 25.2V is 100%, 21.0V is 0%)
  const batteryPct = Math.max(0, Math.min(100, ((voltage - 21.0) / (25.2 - 21.0)) * 100));
  
  const getBatteryColor = () => {
    if (batteryPct >= 30) return 'bg-status-green border-status-green/20';
    if (batteryPct >= 15) return 'bg-status-orange border-status-orange/20';
    return 'bg-destructive border-destructive/20';
  };
  
  const getBatteryTextColor = () => {
    if (batteryPct >= 30) return 'text-status-green';
    if (batteryPct >= 15) return 'text-status-orange';
    return 'text-destructive';
  };
  
  return (
    <Card>
      <CardHeader className="pb-4">
        <CardTitle className="text-lg">System Health</CardTitle>
        <CardDescription>ROS2 SCADA bridge status.</CardDescription>
      </CardHeader>
      
      <CardContent className="flex flex-col gap-6">
        <div className="flex items-center gap-4">
          <div className="p-3.5 rounded-md bg-status-green-bg text-status-green shadow-sm border border-status-green/10">
            <Server size={22} strokeWidth={2.5} aria-hidden="true" />
          </div>
          <div className="flex-1 flex flex-col pt-1">
            <span className="font-bold text-foreground text-sm tracking-tight">ZMQ/IPC Bridge</span>
            <span className="text-[10px] font-bold tracking-[0.15em] text-muted-foreground mt-0.5 uppercase">SCADA Link</span>
          </div>
          {isConnected ? (
             <Badge className="bg-status-green-bg text-status-green border-status-green/20 hover:bg-status-green-bg/80 text-[10px] tracking-widest px-3 py-1 shadow-none">STABLE</Badge>
          ) : (
             <Badge variant="destructive" className="text-[10px] tracking-widest px-3 py-1 shadow-none">OFFLINE</Badge>
          )}
        </div>

        <div className="flex items-center gap-4">
          <div className={`p-3.5 rounded-md shadow-sm border relative overflow-hidden ${getBatteryColor().replace('border-', 'bg-').replace('/20', '/10')} ${getBatteryTextColor()}`}>
            <Zap size={22} strokeWidth={2.5} aria-hidden="true" className="relative z-10" />
            <div className={`absolute inset-0 opacity-20 ${getBatteryColor()} animate-pulse`} />
          </div>
          <div className="flex-1 flex flex-col pt-1 gap-1">
            <div className="flex justify-between items-center">
              <span className="font-bold text-foreground text-sm tracking-tight">24V Battery</span>
              <span className={`font-mono text-sm font-bold flex items-baseline gap-1.5 ${getBatteryTextColor()}`}>
                <span className="text-xs font-medium text-muted-foreground">{voltage.toFixed(1)}V</span>
                {Math.round(batteryPct)}%
              </span>
            </div>
            
            <div className="w-full bg-muted rounded-full h-1.5 mt-1 border border-border overflow-hidden">
               <div className={`h-full ${getBatteryColor().split(' ')[0]} transition-all duration-500`} style={{ width: `${batteryPct}%` }} />
            </div>
          </div>
        </div>

        <div className="flex items-center gap-4">
          <div className="p-3.5 rounded-md bg-status-blue-bg text-status-blue shadow-sm border border-status-blue/10">
            <Cpu size={22} strokeWidth={2.5} aria-hidden="true" />
          </div>
          <div className="flex-1 flex flex-col pt-1">
            <span className="font-bold text-foreground text-sm tracking-tight">DDS Multicast</span>
            <span className="text-[10px] uppercase font-bold tracking-[0.15em] text-muted-foreground mt-0.5">ROS2 Core</span>
          </div>
          {isConnected ? (
             <Badge className="bg-status-blue-bg text-status-blue border-status-blue/20 hover:bg-status-blue-bg/80 text-[10px] tracking-widest px-3 py-1 shadow-none">ACTIVE</Badge>
          ) : (
             <Badge variant="outline" className="text-[10px] tracking-widest px-3 py-1 shadow-none text-muted-foreground">IDLE</Badge>
          )}
        </div>
      </CardContent>
    </Card>
  );
}
