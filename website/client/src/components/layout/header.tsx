'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Badge } from '@/components/ui/badge';

export function Header() {
  const { isConnected, telemetry } = useRobotState();

  return (
    <header className="h-[72px] bg-card text-foreground flex items-center justify-between px-10 border-b border-border shrink-0 z-10 w-full relative">
      <div className="flex items-center gap-4">
        {isConnected ? (
          <Badge className="bg-status-green-bg text-status-green border-status-green/20 hover:bg-status-green-bg/80 shadow-none gap-2 h-8 px-4 text-[11px] font-bold tracking-widest uppercase">
            <span className="relative flex h-2 w-2">
              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-status-green opacity-75" />
              <span className="relative inline-flex rounded-full h-2 w-2 bg-status-green" />
            </span>
            System Online
          </Badge>
        ) : (
          <Badge variant="outline" className="shadow-none gap-2 h-8 px-4 text-[11px] font-bold tracking-widest uppercase text-muted-foreground">
            <span className="relative inline-flex rounded-full h-2 w-2 bg-muted-foreground" />
            Offline
          </Badge>
        )}
      </div>
      
      <div className="flex text-[12px] font-bold tracking-widest uppercase items-center bg-muted p-1 rounded-md border border-border">
        <div className="flex items-center gap-2 px-4 py-1.5 border-r border-border">
          <span className="text-muted-foreground">Bat:</span>
          <span className={`font-mono tabular-nums ${telemetry?.voltage !== undefined && telemetry.voltage <= 21.84 ? 'text-destructive' : 'text-foreground'}`}>
            {telemetry?.voltage !== undefined ? `${telemetry.voltage.toFixed(1)}V` : '--'}
          </span>
        </div>
        <div className="flex items-center gap-2 px-4 py-1.5">
          {telemetry?.charging ? (
            <span className="text-primary font-bold flex items-center gap-1.5">
              <span className="w-1.5 h-1.5 bg-primary rounded-full animate-pulse" /> AC Plugged
            </span>
          ) : (
            <span className="text-muted-foreground">Discharging</span>
          )}
        </div>
      </div>
    </header>
  );
}
