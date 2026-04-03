'use client';

import { useState, useEffect } from 'react';
import { useRobotState } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { RobotMap } from '@/components/map/robot-map';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { Badge } from '@/components/ui/badge';
import { MapPin, Navigation2, Crosshair, Target } from 'lucide-react';
import { toast } from 'sonner';

export default function NavigationPage() {
  const { telemetry, isConnected } = useRobotState();
  
  const currentX = telemetry?.odom?.x || 0;
  const currentY = telemetry?.odom?.y || 0;
  const currentYaw = telemetry?.odom?.yaw || 0;

  // Local state for Waypoint inputs
  const [waypointX, setWaypointX] = useState<string>('0');
  const [waypointY, setWaypointY] = useState<string>('0');
  const [waypointTheta, setWaypointTheta] = useState<string>('0');

  // Home point stored in local state (in a real app, might be stored in DB or ROS parameters)
  const [homePoint, setHomePoint] = useState<{x: number, y: number, theta: number} | null>(null);

  useEffect(() => {
    // Load home point from local storage if exists
    const stored = localStorage.getItem('ros2_home_point');
    if (stored) {
      try {
        setHomePoint(JSON.parse(stored));
      } catch(e) {}
    }
  }, []);

  const handleSaveHome = () => {
    const hp = { x: currentX, y: currentY, theta: currentYaw };
    setHomePoint(hp);
    localStorage.setItem('ros2_home_point', JSON.stringify(hp));
    toast("Home Point Saved", {
      description: `Saved at X: ${hp.x.toFixed(2)}, Y: ${hp.y.toFixed(2)}`,
    });
  };

  const handleGoHome = () => {
    if (!homePoint) {
      toast.error("Error", { description: "No home point set." });
      return;
    }
    rosClient.sendNavGoal(homePoint.x, homePoint.y, homePoint.theta);
    toast("Returning to Home", {
      description: "Navigation goal sent to ROS2.",
    });
  };

  const handleSendWaypoint = () => {
    const x = parseFloat(waypointX);
    const y = parseFloat(waypointY);
    const th = parseFloat(waypointTheta);
    
    if (isNaN(x) || isNaN(y) || isNaN(th)) {
      toast.error("Invalid Input", { description: "Please enter valid numbers." });
      return;
    }

    rosClient.sendNavGoal(x, y, th);
    toast("Waypoint Sent", {
      description: `Target: X=${x}, Y=${y}, Theta=${th}`,
    });
  };

  return (
    <div className="flex flex-col gap-6 min-h-full">
      {/* Page Header */}
      <div className="flex flex-col md:flex-row md:items-center justify-between pb-6 border-b border-border gap-4">
        <div className="flex flex-col">
          <h1 className="text-3xl font-bold tracking-tight text-foreground mb-1">Autonomous Nav</h1>
          <p className="text-sm text-muted-foreground">Manage map targets, waypoints, and home locations.</p>
        </div>
        <Badge variant={isConnected ? "default" : "destructive"} className="w-fit text-[11px] font-bold tracking-widest uppercase px-4 py-2 hover:bg-primary/90">
          {isConnected ? "Nav2 Stack Linked" : "Offline"}
        </Badge>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 items-stretch">
        {/* Left Column: Map */}
        <div className="lg:col-span-8 flex flex-col gap-6 h-full">
          <div className="h-[600px] w-full rounded-md border border-border shadow-sm overflow-hidden bg-card">
            <RobotMap />
          </div>
          <Card>
            <CardHeader className="pb-3">
              <CardTitle className="text-sm border-b pb-2">Telemetry Pose</CardTitle>
            </CardHeader>
            <CardContent>
               <div className="flex justify-between items-center text-sm font-mono text-muted-foreground">
                 <span>X: <strong className="text-foreground">{currentX.toFixed(3)}</strong> m</span>
                 <span>Y: <strong className="text-foreground">{currentY.toFixed(3)}</strong> m</span>
                 <span>Heading: <strong className="text-foreground">{(currentYaw * 180 / Math.PI).toFixed(1)}°</strong></span>
               </div>
            </CardContent>
          </Card>
        </div>

        {/* Right Column: Controls */}
        <div className="lg:col-span-4 flex flex-col gap-6 h-full">
          
          {/* Home Module */}
          <Card className="border-status-blue/20">
            <CardHeader className="pb-4">
              <CardTitle className="flex items-center justify-between">
                 <div className="flex items-center gap-2 text-status-blue">
                   <MapPin size={20} />
                   <span>Home Point</span>
                 </div>
              </CardTitle>
              <CardDescription>Save or return to base.</CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-4">
              <div className="bg-muted p-3 rounded-md font-mono text-xs flex justify-between border">
                {homePoint ? (
                  <>
                    <span>X: {homePoint.x.toFixed(2)}</span>
                    <span>Y: {homePoint.y.toFixed(2)}</span>
                    <span>H: {(homePoint.theta * 180/Math.PI).toFixed(1)}°</span>
                  </>
                ) : (
                  <span className="text-muted-foreground mx-auto">No Home Point Set</span>
                )}
              </div>
              <div className="grid grid-cols-2 gap-3">
                <Button variant="outline" size="sm" onClick={handleSaveHome} disabled={!isConnected}>
                   <Target size={14} className="mr-2" /> Set Current
                </Button>
                <Button size="sm" className="bg-status-blue hover:bg-status-blue/90" onClick={handleGoHome} disabled={!isConnected || !homePoint}>
                   <Navigation2 size={14} className="mr-2" /> Go Home
                </Button>
              </div>
            </CardContent>
          </Card>

          {/* Waypoint Module */}
          <Card>
            <CardHeader className="pb-4">
              <CardTitle className="flex items-center gap-2">
                 <Crosshair size={20} />
                 <span>Manual Waypoint</span>
              </CardTitle>
              <CardDescription>Send a raw coordinate goal to Nav2.</CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-4">
              <div className="grid grid-cols-3 gap-3">
                <div className="flex flex-col gap-2">
                  <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">X (m)</Label>
                  <Input className="font-mono text-xs" value={waypointX} onChange={e => setWaypointX(e.target.value)} />
                </div>
                <div className="flex flex-col gap-2">
                  <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Y (m)</Label>
                  <Input className="font-mono text-xs" value={waypointY} onChange={e => setWaypointY(e.target.value)} />
                </div>
                <div className="flex flex-col gap-2">
                  <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Yaw (rad)</Label>
                  <Input className="font-mono text-xs" value={waypointTheta} onChange={e => setWaypointTheta(e.target.value)} />
                </div>
              </div>
              <Button className="w-full mt-2" onClick={handleSendWaypoint} disabled={!isConnected}>
                <Navigation2 size={14} className="mr-2" /> Send Goal
              </Button>
            </CardContent>
          </Card>

        </div>
      </div>
    </div>
  );
}
