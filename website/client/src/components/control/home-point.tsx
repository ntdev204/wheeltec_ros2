'use client';
import { useEffect } from 'react';
import { useRobotState, TelemetryData } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { MapPin, Navigation, Crosshair } from 'lucide-react';

export function HomePoint() {
  const { telemetry, setTelemetry, isSetHomeMode, setIsSetHomeMode } = useRobotState();
  const home = telemetry?.home_position ?? null;

  // Fetch saved home from REST API on mount
  useEffect(() => {
    const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';
    fetch(`${API_URL}/api/robot/home`)
      .then(r => r.json())
      .then(data => {
        if (data?.home) {
          const home = data.home as NonNullable<TelemetryData['home_position']>;
          setTelemetry((prev) => prev
            ? { ...prev, home_position: home }
            : ({ home_position: home } as TelemetryData)
          );
        }
      })
      .catch(() => {});
  }, [setTelemetry]);

  const handleSetHome = () => {
    setIsSetHomeMode(!isSetHomeMode);
  };

  const handleGoHome = () => {
    if (!home) return;
    rosClient.sendGoHome();
  };

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center gap-2">
          <div className="p-1.5 rounded-md bg-primary/10 text-primary">
            <MapPin size={16} strokeWidth={2.5} />
          </div>
          <div>
            <CardTitle className="text-sm font-bold">Home Point</CardTitle>
            <CardDescription className="text-xs">Save or return to base.</CardDescription>
          </div>
        </div>
      </CardHeader>
      <CardContent className="flex flex-col gap-3">
        {/* Status display */}
        <div className="w-full rounded-lg border border-border bg-muted/30 px-4 py-3 text-center">
          {home ? (
            <span className="text-xs font-bold text-foreground tracking-tight font-mono">
              ({home.x.toFixed(3)}, {home.y.toFixed(3)})
            </span>
          ) : (
            <span className="text-xs font-medium text-muted-foreground">
              No Home Point Set
            </span>
          )}
        </div>

        {/* Action buttons */}
        <div className="grid grid-cols-2 gap-2">
          <button
            onClick={handleSetHome}
            className={`flex items-center justify-center gap-1.5 px-3 py-2.5 rounded-lg border text-xs font-bold tracking-tight transition-all duration-200
              ${isSetHomeMode
                ? 'bg-orange-500/20 text-orange-400 border-orange-500/40'
                : 'bg-background text-foreground border-border hover:bg-muted/80'
              }`}
          >
            <Crosshair size={14} strokeWidth={2.5} />
            {isSetHomeMode ? 'Click Map…' : 'Set Home'}
          </button>
          <button
            onClick={handleGoHome}
            disabled={!home}
            className="flex items-center justify-center gap-1.5 px-3 py-2.5 rounded-lg bg-primary text-primary-foreground text-xs font-bold tracking-tight hover:bg-primary/90 transition-colors disabled:opacity-40 disabled:cursor-not-allowed"
          >
            <Navigation size={14} strokeWidth={2.5} />
            Go Home
          </button>
        </div>
      </CardContent>
    </Card>
  );
}
