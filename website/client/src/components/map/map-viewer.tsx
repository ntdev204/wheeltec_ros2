'use client';
import { useEffect, useRef, useState } from 'react';
import { rosClient } from '@/lib/ros-client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';

export function MapViewer() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { isConnected, telemetry } = useRobotState();
  const [isScanning, setIsScanning] = useState(true); // Default to on for this view
  const [maps, setMaps] = useState<any[]>([]);
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  // Rendering logic for the 2D Map
  useEffect(() => {
    if (!canvasRef.current || !telemetry?.map) return;
    
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const { info, data } = telemetry.map;
    const { width, height, resolution, origin } = info;

    // Adjust canvas size to map resolution
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    // Create ImageData for high-performance pixel drawing
    const imgData = ctx.createImageData(width, height);
    for (let i = 0; i < data.length; i++) {
      const val = data[i];
      const idx = i * 4;
      
      let gray = 255; // White for free space (0)
      let alpha = 255;

      if (val === -1) { // Unknown
        gray = 240; 
        alpha = 150;
      } else if (val > 0) { // Occupied
        gray = 50;
      }

      imgData.data[idx] = gray;
      imgData.data[idx + 1] = gray;
      imgData.data[idx + 2] = gray;
      imgData.data[idx + 3] = alpha;
    }

    ctx.putImageData(imgData, 0, 0);

    // Draw Robot Position (Scanning indicator only)
    const pose = telemetry.map_pose || { x: 0, y: 0, yaw: 0 };
    const robotX = (pose.x - origin.x) / resolution;
    const robotY = height - (pose.y - origin.y) / resolution;
    
    ctx.beginPath();
    ctx.arc(robotX, robotY, 4, 0, 2 * Math.PI);
    ctx.fillStyle = '#ef4444'; // Red for robot
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Heading indicator
    const angle = pose.yaw || 0;
    ctx.beginPath();
    ctx.moveTo(robotX, robotY);
    ctx.lineTo(robotX + Math.cos(angle) * 8, robotY - Math.sin(angle) * 8);
    ctx.strokeStyle = '#ef4444';
    ctx.stroke();

  }, [telemetry?.map, telemetry?.map_pose]);

  const fetchMaps = async () => {
    try {
      const res = await fetch(`${API_URL}/api/maps`);
      const data = await res.json();
      setMaps(data.maps || []);
    } catch (e) {
      console.error("Failed to fetch maps", e);
    }
  };

  useEffect(() => {
    fetchMaps();
  }, [API_URL]);

  const toggleSlam = () => setIsScanning(!isScanning);
  const saveMap = () => alert('Requesting Map Saver on Robot...');

  return (
    <div className="grid grid-cols-1 xl:grid-cols-12 gap-8 h-full min-h-[600px]">
      
      {/* Canvas Area */}
      <Card className="xl:col-span-8 flex flex-col relative overflow-hidden shadow-sm">
        <CardHeader className="flex flex-row items-center justify-between pb-6 border-b border-border/50">
          <div className="flex flex-col gap-1">
             <CardTitle className="text-[11px] font-black text-muted-foreground tracking-[0.2em] uppercase">
               2D Visualizer
             </CardTitle>
             <span className="text-sm font-bold text-foreground">LaserScanner /scan</span>
          </div>
          
          <div className="flex gap-4">
            <Button 
               className={`text-[11px] font-black tracking-widest uppercase h-9 rounded-lg px-6 ${isScanning ? "bg-status-red text-white hover:bg-status-red/90" : "bg-primary text-primary-foreground"}`}
               disabled={!isConnected}
               onClick={toggleSlam}
            >
              {isScanning ? 'Halt Process' : 'Engage Node'}
            </Button>
            <Button 
              variant="outline"
              disabled={!isScanning}
              onClick={saveMap}
              className="text-[11px] font-black tracking-widest uppercase h-9 rounded-lg border-2"
            >
              Capture Grid
            </Button>
          </div>
        </CardHeader>
        
        <CardContent className="flex-1 bg-muted/20 relative flex items-center justify-center p-0 overflow-hidden">
          <div className="w-full h-full p-8 flex items-center justify-center bg-[radial-gradient(#e5e7eb_1px,transparent_1px)] [background-size:16px_16px]">
            <canvas 
              ref={canvasRef} 
              className="max-w-full max-h-full shadow-2xl border border-border/50 bg-white pixelated-canvas" 
              style={{ imageRendering: 'pixelated' }}
            />
          </div>
          
          <div className="absolute top-4 left-4 pointer-events-none">
             {!isScanning && (
               <Badge variant="secondary" className="text-[9px] font-black font-mono tracking-widest bg-background/80 backdrop-blur-md px-3 py-1 shadow-sm uppercase border border-border">
                  Slam Inactive
               </Badge>
             )}
          </div>
        </CardContent>
      </Card>

      {/* Map Database Sidebar */}
      <Card className="xl:col-span-4 flex flex-col overflow-hidden shadow-sm bg-card">
        <CardHeader className="flex flex-row items-center justify-between pb-6 border-b border-border/50">
          <CardTitle className="text-[11px] font-black text-muted-foreground tracking-[0.2em] uppercase">
            Graph Database
          </CardTitle>
          <Button variant="secondary" size="sm" onClick={fetchMaps} className="text-[10px] font-black uppercase h-7 rounded-sm tracking-widest">
            Sync
          </Button>
        </CardHeader>
        
        <CardContent className="flex-1 overflow-auto p-6 flex flex-col gap-4 bg-muted/5">
          {maps.length === 0 ? (
            <div className="flex flex-col items-center justify-center py-20 gap-3 opacity-40">
              <div className="w-12 h-12 rounded-full border-2 border-dashed border-muted-foreground" />
              <div className="text-[10px] font-bold uppercase tracking-[0.2em] text-muted-foreground">SQLite Empty</div>
            </div>
          ) : (
            maps.map((map) => (
              <div key={map.id} className="p-4 border border-border/50 rounded-md bg-background hover:bg-muted/30 transition-all cursor-pointer group flex flex-col gap-3">
                <div className="font-bold text-foreground text-sm tracking-tight flex items-center justify-between">
                  {map.name}
                  {map.is_active === 1 && (
                     <Badge variant="default" className="text-[8px] px-1.5 h-3.5 font-black tracking-widest uppercase bg-status-green text-status-green-bg">Active</Badge>
                  )}
                </div>
                <div className="flex justify-between items-center text-[10px] font-bold text-muted-foreground font-mono tracking-widest uppercase">
                  <span>{map.created_at?.slice(0, 10)}</span>
                  <span className="opacity-0 group-hover:opacity-100 transition-opacity text-primary">Load →</span>
                </div>
              </div>
            ))
          )}
        </CardContent>
      </Card>
    </div>
  );
}
