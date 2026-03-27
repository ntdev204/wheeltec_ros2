'use client';
import { useEffect, useRef, useState } from 'react';
import { useRobotState } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';

interface MapYaml {
  resolution: number;
  origin: [number, number, number];
  image: string;
}

export function RobotMap() {
  const { telemetry } = useRobotState();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [mapYaml, setMapYaml] = useState<MapYaml | null>(null);
  const [mapImage, setMapImage] = useState<HTMLImageElement | null>(null);
  const [navGoal, setNavGoal] = useState<{ x: number, y: number } | null>(null);
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  useEffect(() => {
    // Fetch Map Metadata
    fetch(`${API_URL}/api/maps/static/yaml`)
      .then(res => res.json())
      .then(data => setMapYaml(data))
      .catch(err => console.error("Map YAML fetch error:", err));

    // Fetch Map Image
    const img = new Image();
    img.src = `${API_URL}/api/maps/static/image`;
    img.onload = () => setMapImage(img);
  }, [API_URL]);

  useEffect(() => {
    if (!canvasRef.current || !mapImage || !mapYaml) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear and draw map
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Maintain aspect ratio while fitting into canvas
    const scale = Math.min(canvas.width / mapImage.width, canvas.height / mapImage.height);
    const x = (canvas.width - mapImage.width * scale) / 2;
    const y = (canvas.height - mapImage.height * scale) / 2;
    
    ctx.drawImage(mapImage, x, y, mapImage.width * scale, mapImage.height * scale);

    // Draw Robot if telemetry exists
    // Fallback order: map_pose (AMCL) -> odom (Raw)
    const pose = telemetry?.map_pose || (telemetry?.odom ? { x: telemetry.odom.x, y: telemetry.odom.y, yaw: telemetry.odom.yaw || 0, isFallback: true } : null);
    
    if (pose && mapYaml) {
      const { x: rx, y: ry, yaw: rz } = pose;
      
      // Convert ROS (meters) to Pixels
      const rx_px = (rx - mapYaml.origin[0]) / mapYaml.resolution;
      const ry_px = mapImage.height - (ry - mapYaml.origin[1]) / mapYaml.resolution;

      // Map Pixels to Canvas coordinates (with scale/offset)
      const cx = x + rx_px * scale;
      const cy = y + ry_px * scale;

      // Draw Robot Marker
      ctx.beginPath();
      ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      ctx.fillStyle = '#ef4444'; // Use consistent red for locator
      ctx.shadowBlur = 8;
      ctx.shadowColor = 'rgba(239, 68, 68, 0.4)';
      ctx.fill();
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.shadowBlur = 0;

      // Draw Heading Arrow
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      const arrowLen = 12;
      ctx.lineTo(
        cx + Math.cos(rz) * arrowLen,
        cy - Math.sin(rz) * arrowLen 
      );
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Draw Nav Goal (Flag/Marker) if exists
    if (navGoal && mapYaml) {
      const gx_px = (navGoal.x - mapYaml.origin[0]) / mapYaml.resolution;
      const gy_px = mapImage.height - (navGoal.y - mapYaml.origin[1]) / mapYaml.resolution;
      const gcx = x + gx_px * scale;
      const gcy = y + gy_px * scale;

      // Draw Goal Cross
      ctx.beginPath();
      const crossSize = 8;
      ctx.moveTo(gcx - crossSize, gcy - crossSize);
      ctx.lineTo(gcx + crossSize, gcy + crossSize);
      ctx.moveTo(gcx + crossSize, gcy - crossSize);
      ctx.lineTo(gcx - crossSize, gcy + crossSize);
      ctx.strokeStyle = '#22c55e'; // Green for goal
      ctx.lineWidth = 3;
      ctx.stroke();

      // Halo effect
      ctx.beginPath();
      ctx.arc(gcx, gcy, 12, 0, Math.PI * 2);
      ctx.strokeStyle = '#22c55e55';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

  }, [mapImage, mapYaml, telemetry, navGoal]);

  const handleMapClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!canvasRef.current || !mapImage || !mapYaml) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    
    // Calculate click relative to map image area (accounting for object-contain scale)
    const scale = Math.min(canvas.width / mapImage.width, canvas.height / mapImage.height);
    const offsetX = (canvas.width - mapImage.width * scale) / 2;
    const offsetY = (canvas.height - mapImage.height * scale) / 2;

    const clickX_canvas = (e.clientX - rect.left) * (canvas.width / rect.width);
    const clickY_canvas = (e.clientY - rect.top) * (canvas.height / rect.height);

    // Convert canvas click to map image pixels
    const px = (clickX_canvas - offsetX) / scale;
    const py = (clickY_canvas - offsetY) / scale;

    if (px < 0 || px > mapImage.width || py < 0 || py > mapImage.height) return;

    // Convert map pixels to ROS meters
    const mx = (px * mapYaml.resolution) + mapYaml.origin[0];
    const my = ((mapImage.height - py) * mapYaml.resolution) + mapYaml.origin[1];

    // Send Nav Goal
    rosClient.send('nav_goal', { x: mx, y: my });
    setNavGoal({ x: mx, y: my });
    console.log(`Sending Omni goal: x=${mx}, y=${my}`);
  };

  return (
    <Card className="w-full flex flex-col shadow-sm border-border h-full overflow-hidden">
      <CardHeader className="pb-4 flex flex-row items-center justify-between border-b border-border/50">
        <div className="flex flex-col">
          <CardTitle className="text-[11px] font-bold text-muted-foreground tracking-[0.2em] uppercase">
            Robot Locator
          </CardTitle>
          <span className="text-xs font-bold text-foreground mt-1">Global Occupancy Grid</span>
        </div>
        {mapYaml && (
          <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-[9px] font-mono font-bold uppercase tracking-widest text-muted-foreground/80">
            <div className="flex justify-between gap-2"><span>Res:</span> <span className="text-foreground">{mapYaml.resolution}m</span></div>
            <div className="flex justify-between gap-2"><span>Origin:</span> <span className="text-foreground">{mapYaml.origin[0]},{mapYaml.origin[1]}</span></div>
          </div>
        )}
      </CardHeader>
      <CardContent className="flex-1 bg-muted/10 relative flex items-center justify-center p-0 overflow-hidden">
        <canvas 
          ref={canvasRef} 
          width={1000} 
          height={750} 
          onClick={handleMapClick}
          className="w-full h-full object-contain cursor-crosshair hover:opacity-90 active:scale-[0.99] transition-all"
        />
        {!telemetry && (
          <div className="absolute inset-0 flex items-center justify-center bg-background/50 backdrop-blur-md transition-all duration-500">
             <div className="flex flex-col items-center gap-3">
               <div className="w-8 h-8 rounded-full border-2 border-primary border-t-transparent animate-spin" />
               <span className="text-[10px] font-black tracking-widest uppercase text-muted-foreground">Synchronizing Odom…</span>
             </div>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
