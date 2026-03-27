'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { Server, Zap, Cpu } from 'lucide-react';
import { Badge } from '@/components/ui/badge';

export function SystemHealth() {
  const { isConnected, telemetry } = useRobotState();

  const voltage = telemetry?.voltage ?? 0.0;
  
  return (
    <Card>
      <CardHeader>
        <CardTitle>System Health</CardTitle>
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
          <div className="p-3.5 rounded-md bg-status-orange-bg text-status-orange shadow-sm border border-status-orange/10">
            <Zap size={22} strokeWidth={2.5} aria-hidden="true" />
          </div>
          <div className="flex-1 flex flex-col pt-1">
            <span className="font-bold text-foreground text-sm tracking-tight">Power Source</span>
            <span className="text-[10px] uppercase font-bold tracking-[0.15em] text-muted-foreground mt-0.5">{telemetry?.charging ? 'Main AC' : 'DC Battery'}</span>
          </div>
          <Badge className="bg-status-orange-bg text-status-orange border-status-orange/20 hover:bg-status-orange-bg/80 text-[11px] font-mono tracking-widest px-3 py-1 shadow-none">
            {voltage.toFixed(1)}V
          </Badge>
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
