'use client';
import { useEffect, useRef, useState } from 'react';
import { rosClient } from '@/lib/ros-client';
import { useRobotState } from '@/hooks/use-robot-state';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';

export function MapViewer() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { isConnected } = useRobotState();
  const [isScanning, setIsScanning] = useState(false);
  const [maps, setMaps] = useState<any[]>([]);
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

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
  const saveMap = () => alert('Saving graph...');

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
              variant={isScanning ? "destructive" : "default"}
              disabled={!isConnected}
              onClick={toggleSlam}
              className="text-[11px] font-black tracking-widest uppercase h-9 rounded-lg"
            >
              {isScanning ? 'Halt Process' : 'Engage Node'}
            </Button>
            <Button 
              variant="outline"
              disabled={!isScanning}
              onClick={saveMap}
              className="text-[11px] font-black tracking-widest uppercase h-9 rounded-lg"
            >
              Capture Grid
            </Button>
          </div>
        </CardHeader>
        
        <CardContent className="flex-1 bg-muted/30 relative flex items-center justify-center p-8">
          <canvas ref={canvasRef} className="w-full h-full max-h-[500px] border border-border shadow-sm rounded-md bg-background" />
          
          <div className="absolute inset-0 pointer-events-none flex items-center justify-center">
             {!isScanning && <div className="text-[10px] text-muted-foreground font-black font-mono tracking-[0.2em] bg-background px-6 py-3 rounded-full border border-border shadow-sm uppercase">Slam Inactive</div>}
          </div>
        </CardContent>
      </Card>

      {/* Map Database Sidebar */}
      <Card className="xl:col-span-4 flex flex-col overflow-hidden shadow-sm">
        <CardHeader className="flex flex-row items-center justify-between pb-6 border-b border-border/50">
          <CardTitle className="text-[11px] font-black text-muted-foreground tracking-[0.2em] uppercase">
            Graph Database
          </CardTitle>
          <Button variant="secondary" size="sm" onClick={fetchMaps} className="text-[10px] font-black uppercase h-7 rounded-sm tracking-widest">
            Sync
          </Button>
        </CardHeader>
        
        <CardContent className="flex-1 overflow-auto p-6 flex flex-col gap-4 bg-muted/10">
          {maps.length === 0 ? (
            <div className="text-center text-[10px] font-bold uppercase tracking-[0.2em] text-muted-foreground my-8">SQLite Empty</div>
          ) : (
            maps.map((map) => (
              <div key={map.id} className="p-4 border border-border rounded-md bg-background hover:bg-muted/50 hover:border-primary transition-all cursor-pointer group flex flex-col gap-3 shadow-sm">
                <div className="font-bold text-foreground text-sm tracking-tight">{map.name}</div>
                <div className="flex justify-between items-center">
                  <span className="text-[10px] font-bold text-muted-foreground font-mono tracking-widest uppercase">{map.created_at?.slice(0, 10)}</span>
                  {map.is_active === 1 && (
                     <Badge variant="default" className="text-[9px] px-2 py-0.5 h-4 font-black tracking-widest uppercase shadow-none bg-status-green-bg text-status-green hover:bg-status-green-bg">Active</Badge>
                  )}
                </div>
              </div>
            ))
          )}
        </CardContent>
      </Card>
    </div>
  );
}
