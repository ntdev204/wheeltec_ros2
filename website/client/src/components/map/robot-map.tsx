'use client';
import { useEffect, useRef, useState, useCallback } from 'react';
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
  const [navGoal, setNavGoal] = useState<{ x: number; y: number } | null>(null);
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  // Load static PGM map + YAML metadata (one-time fetch)
  useEffect(() => {
    fetch(`${API_URL}/api/maps/static/yaml`)
      .then(res => res.json())
      .then(data => setMapYaml(data))
      .catch(err => console.error('YAML fetch error:', err));

    const img = new Image();
    img.src = `${API_URL}/api/maps/static/image`;
    img.onload = () => setMapImage(img);
  }, [API_URL]);

  // === COORDINATE HELPERS (PGM + YAML based) ===
  // ROS meters -> PGM image pixels
  const rosToPixel = useCallback((rosX: number, rosY: number) => {
    if (!mapYaml || !mapImage) return { px: 0, py: 0 };
    const px = (rosX - mapYaml.origin[0]) / mapYaml.resolution;
    const py = (mapImage.height - 1) - (rosY - mapYaml.origin[1]) / mapYaml.resolution;
    return { px, py };
  }, [mapYaml, mapImage]);

  // PGM image pixels -> ROS meters
  const pixelToRos = useCallback((px: number, py: number) => {
    if (!mapYaml || !mapImage) return { rosX: 0, rosY: 0 };
    const rosX = px * mapYaml.resolution + mapYaml.origin[0];
    const rosY = ((mapImage.height - 1) - py) * mapYaml.resolution + mapYaml.origin[1];
    return { rosX, rosY };
  }, [mapYaml, mapImage]);

  // Render map + robot + goal
  useEffect(() => {
    if (!canvasRef.current || !mapImage || !mapYaml) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Match canvas to image size
    if (canvas.width !== mapImage.width || canvas.height !== mapImage.height) {
      canvas.width = mapImage.width;
      canvas.height = mapImage.height;
    }

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(mapImage, 0, 0);

    // Draw Robot (from TF2 map->base_link lookup)
    const pose = telemetry?.map_pose;

    if (pose && (pose.x !== 0 || pose.y !== 0)) {
      const { px, py } = rosToPixel(pose.x, pose.y);

      ctx.beginPath();
      ctx.arc(px, py, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#ef4444';
      ctx.shadowBlur = 6;
      ctx.shadowColor = 'rgba(239,68,68,0.5)';
      ctx.fill();
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.shadowBlur = 0;

      // Heading arrow
      const arrowLen = 10;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(pose.yaw) * arrowLen, py - Math.sin(pose.yaw) * arrowLen);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Draw Nav Goal
    if (navGoal) {
      const { px, py } = rosToPixel(navGoal.x, navGoal.y);
      const cs = 6;
      ctx.beginPath();
      ctx.moveTo(px - cs, py - cs);
      ctx.lineTo(px + cs, py + cs);
      ctx.moveTo(px + cs, py - cs);
      ctx.lineTo(px - cs, py + cs);
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(px, py, 10, 0, Math.PI * 2);
      ctx.strokeStyle = '#22c55e66';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
  }, [mapImage, mapYaml, telemetry, navGoal, rosToPixel]);

  // Click → Nav Goal
  const handleMapClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!canvasRef.current || !mapImage || !mapYaml) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();

    // CSS pixel → image pixel
    const imgPx = (e.clientX - rect.left) * (canvas.width / rect.width);
    const imgPy = (e.clientY - rect.top) * (canvas.height / rect.height);

    if (imgPx < 0 || imgPx >= mapImage.width || imgPy < 0 || imgPy >= mapImage.height) return;

    const { rosX, rosY } = pixelToRos(imgPx, imgPy);
    console.log(`[NAV] pixel=(${imgPx.toFixed(1)},${imgPy.toFixed(1)}) → ROS=(${rosX.toFixed(3)},${rosY.toFixed(3)})`);

    rosClient.send('nav_goal', { x: rosX, y: rosY });
    setNavGoal({ x: rosX, y: rosY });
  };

  return (
    <Card className="w-full flex flex-col shadow-sm border-border h-full overflow-hidden">
      <CardHeader className="pb-4 flex flex-row items-center justify-between border-b border-border/50">
        <div className="flex flex-col">
          <CardTitle className="text-[11px] font-bold text-muted-foreground tracking-[0.2em] uppercase">
            Robot Locator
          </CardTitle>
          <span className="text-xs font-bold text-foreground mt-1">Static Map + TF2 Pose</span>
        </div>
        {mapYaml && (
          <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-[9px] font-mono font-bold uppercase tracking-widest text-muted-foreground/80">
            <div className="flex justify-between gap-2"><span>Res:</span> <span className="text-foreground">{mapYaml.resolution}m</span></div>
            <div className="flex justify-between gap-2"><span>Origin:</span> <span className="text-foreground">{mapYaml.origin[0]},{mapYaml.origin[1]}</span></div>
            {mapImage && (
              <div className="flex justify-between gap-2"><span>Size:</span> <span className="text-foreground">{mapImage.width}×{mapImage.height}</span></div>
            )}
            {navGoal && (
              <div className="flex justify-between gap-2"><span>Goal:</span> <span className="text-green-500">{navGoal.x.toFixed(2)},{navGoal.y.toFixed(2)}</span></div>
            )}
          </div>
        )}
      </CardHeader>
      <CardContent className="flex-1 bg-muted/10 relative flex items-center justify-center p-0 overflow-hidden">
        <canvas
          ref={canvasRef}
          onClick={handleMapClick}
          className="w-full h-full object-contain cursor-crosshair"
          style={{ imageRendering: 'pixelated' }}
        />
        {!mapImage && (
          <div className="absolute inset-0 flex items-center justify-center bg-background/50 backdrop-blur-md">
            <div className="flex flex-col items-center gap-3">
              <div className="w-8 h-8 rounded-full border-2 border-primary border-t-transparent animate-spin" />
              <span className="text-[10px] font-black tracking-widest uppercase text-muted-foreground">Loading map…</span>
            </div>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
