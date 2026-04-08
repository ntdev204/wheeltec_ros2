'use client';

import { useEffect, useMemo, useState } from 'react';
import { toast } from 'sonner';
import { AlarmClock, LoaderCircle, MapPinned, Route, Save, Square, Play } from 'lucide-react';

import { useRobotState, PatrolRouteWaypoint } from '@/hooks/use-robot-state';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { Switch } from '@/components/ui/switch';

const DEFAULT_ROUTE_NAME = 'Patrol Route';

function createWaypoint(x = 0, y = 0, yaw = 0): PatrolRouteWaypoint {
  return { x, y, yaw };
}

export function PatrolPanel() {
  const { patrolStatus, isConnected } = useRobotState();
  const route = patrolStatus?.route ?? null;
  const schedule = patrolStatus?.schedule ?? null;
  const runtime = patrolStatus?.runtime ?? null;
  const latestRun = patrolStatus?.latest_run ?? null;

  const [routeName, setRouteName] = useState(DEFAULT_ROUTE_NAME);
  const [waypoints, setWaypoints] = useState<PatrolRouteWaypoint[]>([createWaypoint(), createWaypoint(1, 0, 0)]);
  const [intervalMinutes, setIntervalMinutes] = useState(30);
  const [loopsPerRun, setLoopsPerRun] = useState(5);
  const [enabled, setEnabled] = useState(false);
  const [isSavingRoute, setIsSavingRoute] = useState(false);
  const [isSavingSchedule, setIsSavingSchedule] = useState(false);
  const [isStarting, setIsStarting] = useState(false);
  const [isStopping, setIsStopping] = useState(false);

  const apiUrl = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  useEffect(() => {
    if (!route) {
      return;
    }
    setRouteName(route.name);
    setWaypoints(route.waypoints.length >= 2 ? route.waypoints : [createWaypoint(), createWaypoint(1, 0, 0)]);
  }, [route]);

  useEffect(() => {
    if (!schedule) {
      return;
    }
    setIntervalMinutes(schedule.interval_minutes);
    setLoopsPerRun(schedule.loops_per_run);
    setEnabled(schedule.enabled);
  }, [schedule]);

  const hasValidRoute = useMemo(() => waypoints.length >= 2, [waypoints]);
  const runtimeStatus = runtime?.status ?? 'idle';
  const isMissionActive = ['starting', 'running', 'returning_home'].includes(runtimeStatus);

  const updateWaypoint = (index: number, key: keyof PatrolRouteWaypoint, value: string) => {
    const parsed = Number.parseFloat(value);
    setWaypoints((prev) => prev.map((waypoint, waypointIndex) => (
      waypointIndex === index
        ? { ...waypoint, [key]: Number.isFinite(parsed) ? parsed : 0 }
        : waypoint
    )));
  };

  const addWaypoint = () => {
    setWaypoints((prev) => [...prev, createWaypoint()]);
  };

  const removeWaypoint = (index: number) => {
    setWaypoints((prev) => prev.length <= 2 ? prev : prev.filter((_, waypointIndex) => waypointIndex !== index));
  };

  const saveRoute = async () => {
    setIsSavingRoute(true);
    try {
      const response = await fetch(`${apiUrl}/api/robot/patrol/route`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: routeName || DEFAULT_ROUTE_NAME, waypoints }),
      });
      const body = await response.json();
      if (!response.ok) {
        throw new Error(body?.detail || 'Failed to save patrol route');
      }
      toast.success('Patrol route saved');
    } catch (error) {
      toast.error('Route save failed', { description: error instanceof Error ? error.message : 'Unknown error' });
    } finally {
      setIsSavingRoute(false);
    }
  };

  const saveSchedule = async () => {
    setIsSavingSchedule(true);
    try {
      const response = await fetch(`${apiUrl}/api/robot/patrol/schedule`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          enabled,
          interval_minutes: intervalMinutes,
          loops_per_run: loopsPerRun,
          route_id: route?.id ?? null,
        }),
      });
      const body = await response.json();
      if (!response.ok) {
        throw new Error(body?.detail || 'Failed to save schedule');
      }
      toast.success('Patrol schedule updated');
    } catch (error) {
      toast.error('Schedule update failed', { description: error instanceof Error ? error.message : 'Unknown error' });
    } finally {
      setIsSavingSchedule(false);
    }
  };

  const startPatrol = async () => {
    setIsStarting(true);
    try {
      const response = await fetch(`${apiUrl}/api/robot/patrol/start`, { method: 'POST' });
      const body = await response.json();
      if (!response.ok) {
        throw new Error(body?.detail || 'Failed to start patrol');
      }
      toast.success('Patrol started');
    } catch (error) {
      toast.error('Unable to start patrol', { description: error instanceof Error ? error.message : 'Unknown error' });
    } finally {
      setIsStarting(false);
    }
  };

  const stopPatrol = async () => {
    setIsStopping(true);
    try {
      const response = await fetch(`${apiUrl}/api/robot/patrol/stop`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ reason: 'Stopped from navigation UI' }),
      });
      const body = await response.json();
      if (!response.ok) {
        throw new Error(body?.detail || 'Failed to stop patrol');
      }
      toast.success('Patrol stopped');
    } catch (error) {
      toast.error('Unable to stop patrol', { description: error instanceof Error ? error.message : 'Unknown error' });
    } finally {
      setIsStopping(false);
    }
  };

  return (
    <div className="flex flex-col gap-6">
      <Card>
        <CardHeader className="pb-4">
          <CardTitle className="flex items-center gap-2 text-sm">
            <Route size={18} />
            Patrol Route
          </CardTitle>
          <CardDescription>Configure the ordered waypoint loop used for scheduled patrol missions.</CardDescription>
        </CardHeader>
        <CardContent className="flex flex-col gap-4">
          <div className="flex flex-col gap-2">
            <Label htmlFor="patrol-route-name">Route name</Label>
            <Input id="patrol-route-name" value={routeName} onChange={(event) => setRouteName(event.target.value)} />
          </div>

          <div className="flex flex-col gap-3">
            {waypoints.map((waypoint, index) => (
              <div key={`waypoint-${index}`} className="grid grid-cols-4 gap-2 items-end rounded-lg border border-border/60 p-3">
                <div className="flex flex-col gap-2">
                  <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">X</Label>
                  <Input value={String(waypoint.x)} onChange={(event) => updateWaypoint(index, 'x', event.target.value)} />
                </div>
                <div className="flex flex-col gap-2">
                  <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Y</Label>
                  <Input value={String(waypoint.y)} onChange={(event) => updateWaypoint(index, 'y', event.target.value)} />
                </div>
                <div className="flex flex-col gap-2">
                  <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Yaw</Label>
                  <Input value={String(waypoint.yaw)} onChange={(event) => updateWaypoint(index, 'yaw', event.target.value)} />
                </div>
                <Button type="button" variant="outline" onClick={() => removeWaypoint(index)} disabled={waypoints.length <= 2}>
                  Remove
                </Button>
              </div>
            ))}
          </div>

          <div className="flex gap-2">
            <Button type="button" variant="outline" onClick={addWaypoint}>Add waypoint</Button>
            <Button type="button" onClick={saveRoute} disabled={!hasValidRoute || isSavingRoute}>
              {isSavingRoute ? <LoaderCircle size={14} className="mr-2 animate-spin" /> : <Save size={14} className="mr-2" />}
              Save route
            </Button>
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader className="pb-4">
          <CardTitle className="flex items-center gap-2 text-sm">
            <AlarmClock size={18} />
            Patrol Schedule
          </CardTitle>
          <CardDescription>Run the patrol route every 30 minutes, 5 loops per mission, starting and ending at home.</CardDescription>
        </CardHeader>
        <CardContent className="flex flex-col gap-4">
          <div className="flex items-center justify-between rounded-lg border border-border/60 p-3">
            <div className="flex flex-col gap-1">
              <span className="text-sm font-medium">Enable patrol schedule</span>
              <span className="text-xs text-muted-foreground">The backend scheduler triggers a patrol mission when all safety conditions pass.</span>
            </div>
            <Switch checked={enabled} onCheckedChange={setEnabled} />
          </div>

          <div className="grid grid-cols-2 gap-3">
            <div className="flex flex-col gap-2">
              <Label htmlFor="interval-minutes">Interval minutes</Label>
              <Input id="interval-minutes" type="number" min={1} value={intervalMinutes} onChange={(event) => setIntervalMinutes(Number.parseInt(event.target.value || '0', 10) || 0)} />
            </div>
            <div className="flex flex-col gap-2">
              <Label htmlFor="loops-per-run">Loops per run</Label>
              <Input id="loops-per-run" type="number" min={1} value={loopsPerRun} onChange={(event) => setLoopsPerRun(Number.parseInt(event.target.value || '0', 10) || 0)} />
            </div>
          </div>

          <Button type="button" onClick={saveSchedule} disabled={isSavingSchedule || !route}>
            {isSavingSchedule ? <LoaderCircle size={14} className="mr-2 animate-spin" /> : <Save size={14} className="mr-2" />}
            Save schedule
          </Button>
        </CardContent>
      </Card>

      <Card>
        <CardHeader className="pb-4">
          <CardTitle className="flex items-center gap-2 text-sm">
            <MapPinned size={18} />
            Patrol Runtime
          </CardTitle>
          <CardDescription>Monitor the active patrol mission and manually start or stop runs.</CardDescription>
        </CardHeader>
        <CardContent className="flex flex-col gap-4">
          <div className="grid grid-cols-2 gap-3 rounded-lg border border-border/60 p-3 text-sm">
            <div className="flex flex-col gap-1">
              <span className="text-muted-foreground">Runtime status</span>
              <span className="font-semibold capitalize">{runtimeStatus.replaceAll('_', ' ')}</span>
            </div>
            <div className="flex flex-col gap-1">
              <span className="text-muted-foreground">Current loop</span>
              <span className="font-semibold">{runtime?.current_loop ?? 0} / {runtime?.total_loops ?? 0}</span>
            </div>
            <div className="flex flex-col gap-1">
              <span className="text-muted-foreground">Waypoint</span>
              <span className="font-semibold">{(runtime?.current_waypoint_index ?? -1) + 1} / {runtime?.total_waypoints ?? 0}</span>
            </div>
            <div className="flex flex-col gap-1">
              <span className="text-muted-foreground">Next trigger</span>
              <span className="font-semibold">{schedule?.next_trigger_at ? new Date(schedule.next_trigger_at).toLocaleString() : '--'}</span>
            </div>
          </div>

          <div className="rounded-lg border border-border/60 p-3 text-sm">
            <div className="text-muted-foreground">Latest mission</div>
            <div className="mt-1 font-semibold">{latestRun ? `#${latestRun.id} · ${latestRun.status}` : 'No patrol mission yet'}</div>
            {runtime?.message ? <div className="mt-2 text-xs text-muted-foreground">{runtime.message}</div> : null}
          </div>

          <div className="flex gap-2">
            <Button type="button" onClick={startPatrol} disabled={!isConnected || !route || isMissionActive || isStarting}>
              {isStarting ? <LoaderCircle size={14} className="mr-2 animate-spin" /> : <Play size={14} className="mr-2" />}
              Start now
            </Button>
            <Button type="button" variant="outline" onClick={stopPatrol} disabled={!isMissionActive || isStopping}>
              {isStopping ? <LoaderCircle size={14} className="mr-2 animate-spin" /> : <Square size={14} className="mr-2" />}
              Stop patrol
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
