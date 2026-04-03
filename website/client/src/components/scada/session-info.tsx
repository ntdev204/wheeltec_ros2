'use client';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { Database, TrendingUp, AlertTriangle } from 'lucide-react';
import { format } from 'date-fns';

export function SessionInfo({ sessionData, logStats }: { sessionData: any, logStats: any }) {
  const startedAt = sessionData?.started_at ? new Date(sessionData.started_at) : null;
  const timeStr = startedAt ? format(startedAt, 'HH:mm:ss dd/MM') : '--';
  
  const distance = sessionData?.total_distance || 0;
  const maxSpeed = sessionData?.max_speed || 0;
  const eStops = sessionData?.emergency_stops || 0;
  
  const logCount = logStats?.total || 0;
  
  return (
    <Card>
      <CardHeader className="pb-4">
        <CardTitle className="text-lg flex items-center justify-between">
          <span>Active Session</span>
          <span className="text-[10px] bg-primary/10 text-primary px-2 py-0.5 rounded-sm font-mono tracking-widest uppercase">ID: {sessionData?.id || '--'}</span>
        </CardTitle>
        <CardDescription>Metrics tracked since boot.</CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-4">
        
        <div className="flex items-center gap-4">
           <div className="p-3 bg-muted rounded-md text-muted-foreground border border-border/50">
             <TrendingUp size={16} />
           </div>
           <div className="flex-1 flex justify-between items-center border-b border-border/50 pb-2">
             <span className="text-sm font-bold text-muted-foreground tracking-wide">MAX SPEED</span>
             <span className="font-mono text-foreground font-bold">{maxSpeed.toFixed(2)} m/s</span>
           </div>
        </div>
        
        <div className="flex items-center gap-4">
           <div className="p-3 bg-status-orange-bg text-status-orange rounded-md border border-status-orange/20">
             <AlertTriangle size={16} />
           </div>
           <div className="flex-1 flex justify-between items-center border-b border-border/50 pb-2">
             <span className="text-sm font-bold text-muted-foreground tracking-wide">E-STOPS</span>
             <span className="font-mono text-status-orange font-bold">{eStops}</span>
           </div>
        </div>
        
        <div className="flex items-center gap-4">
           <div className="p-3 bg-status-blue-bg text-status-blue rounded-md border border-status-blue/20">
             <Database size={16} />
           </div>
           <div className="flex-1 flex justify-between items-center border-b border-border/50 pb-2">
             <span className="text-sm font-bold text-muted-foreground tracking-wide">LOGS SAVED</span>
             <span className="font-mono text-foreground font-bold">{logCount}</span>
           </div>
        </div>

        <div className="mt-2 text-center text-xs text-muted-foreground font-mono bg-muted/30 py-2 rounded-md">
          Started: {timeStr}
        </div>
      </CardContent>
    </Card>
  );
}
