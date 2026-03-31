'use client';
import { useEffect, useRef, useState, useCallback } from 'react';
import { useRobotState } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';

interface MapInfo {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
}

export function RobotMap() {
  const { telemetry } = useRobotState();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [navGoal, setNavGoal] = useState<{ x: number; y: number } | null>(null);
  const [mapImage, setMapImage] = useState<HTMLImageElement | null>(null);
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  // Get map_info from telemetry (sent in regular telemetry stream, always up to date)
  const mapInfo: MapInfo | null = telemetry?.map_info || null;

  // Poll live map PNG from HTTP API every 3 seconds
  useEffect(() => {
    let cancelled = false;

    const fetchMapImage = async () => {
      try {
        const res = await fetch(`${API_URL}/api/maps/live/image?t=${Date.now()}`);
        if (res.ok) {
          const blob = await res.blob();
          const url = URL.createObjectURL(blob);
          const img = new Image();
          img.onload = () => { if (!cancelled) setMapImage(img); };
          img.src = url;
        } else if (res.status === 404) {
          // Map chưa có trong cache, yêu cầu robot gửi lại
          fetch(`${API_URL}/api/maps/live/trigger`, { method: 'POST' }).catch(() => {});
        }
      } catch (e) {
        // Network error
      }
    };

    fetchMapImage();
    const interval = setInterval(fetchMapImage, 3000);
    return () => { cancelled = true; clearInterval(interval); };
  }, [API_URL]);

  // COORDINATE HELPERS
  // PNG is already flipped (np.flipud on server), so:
  //   canvas row 0 = map top = highest Y in ROS frame
  //   px = (rosX - origin.x) / resolution
  //   py = (height - 1) - (rosY - origin.y) / resolution  → same formula, matches flipped PNG
  const rosToPixel = useCallback((rosX: number, rosY: number) => {
    if (!mapInfo) return { px: 0, py: 0 };
    const px = (rosX - mapInfo.origin.x) / mapInfo.resolution;
    const py = (mapInfo.height - 1) - (rosY - mapInfo.origin.y) / mapInfo.resolution;
    return { px, py };
  }, [mapInfo]);

  const pixelToRos = useCallback((px: number, py: number) => {
    if (!mapInfo) return { rosX: 0, rosY: 0 };
    const rosX = px * mapInfo.resolution + mapInfo.origin.x;
    const rosY = ((mapInfo.height - 1) - py) * mapInfo.resolution + mapInfo.origin.y;
    return { rosX, rosY };
  }, [mapInfo]);

  // Render
  useEffect(() => {
    if (!canvasRef.current || !mapImage || !mapInfo) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    if (canvas.width !== mapInfo.width || canvas.height !== mapInfo.height) {
      canvas.width = mapInfo.width;
      canvas.height = mapInfo.height;
    }

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(mapImage, 0, 0, mapInfo.width, mapInfo.height);

    // Draw Robot
    const pose = telemetry?.map_pose || (telemetry?.odom ? { x: telemetry.odom.x, y: telemetry.odom.y, yaw: telemetry.odom.yaw || 0 } : null);

    if (pose) {
      const { px, py } = rosToPixel(pose.x, pose.y);
      ctx.beginPath();
      ctx.arc(px, py, 4, 0, Math.PI * 2);
      ctx.fillStyle = '#ef4444';
      ctx.fill();
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 1;
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(pose.yaw) * 8, py - Math.sin(pose.yaw) * 8);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Draw Nav Goal
    if (navGoal) {
      const { px, py } = rosToPixel(navGoal.x, navGoal.y);
      const cs = 5;
      ctx.beginPath();
      ctx.moveTo(px - cs, py - cs);
      ctx.lineTo(px + cs, py + cs);
      ctx.moveTo(px + cs, py - cs);
      ctx.lineTo(px - cs, py + cs);
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(px, py, 8, 0, Math.PI * 2);
      ctx.strokeStyle = '#22c55e88';
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }, [mapImage, mapInfo, telemetry, navGoal, rosToPixel]);

  const handleMapClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!canvasRef.current || !mapInfo) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const imgPx = (e.clientX - rect.left) * (canvas.width / rect.width);
    const imgPy = (e.clientY - rect.top) * (canvas.height / rect.height);
    if (imgPx < 0 || imgPx >= mapInfo.width || imgPy < 0 || imgPy >= mapInfo.height) return;

    const { rosX, rosY } = pixelToRos(imgPx, imgPy);
    console.log(`[NAV] pixel=(${imgPx.toFixed(1)}, ${imgPy.toFixed(1)}) -> ROS=(${rosX.toFixed(3)}, ${rosY.toFixed(3)})`);
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
          <span className="text-xs font-bold text-foreground mt-1">Nav2 Live Costmap</span>
        </div>
        {mapInfo && (
          <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-[9px] font-mono font-bold uppercase tracking-widest text-muted-foreground/80">
            <div className="flex justify-between gap-2"><span>Size:</span> <span className="text-foreground">{mapInfo.width}×{mapInfo.height}</span></div>
            <div className="flex justify-between gap-2"><span>Res:</span> <span className="text-foreground">{mapInfo.resolution}m</span></div>
          </div>
        )}
      </CardHeader>
      <CardContent className="flex-1 bg-muted/10 relative p-0 overflow-hidden">
        {/* Wrapper fills the card; canvas is scaled via CSS to fit while keeping aspect ratio */}
        <div className="absolute inset-0 flex items-center justify-center">
          <canvas
            ref={canvasRef}
            onClick={handleMapClick}
            className="cursor-crosshair"
            style={{
              imageRendering: 'pixelated',
              maxWidth: '100%',
              maxHeight: '100%',
              width: 'auto',
              height: 'auto',
              display: 'block',
            }}
          />
        </div>
        {!mapImage && (
          <div className="absolute inset-0 flex items-center justify-center bg-background/50 backdrop-blur-md">
            <div className="flex flex-col items-center gap-3">
              <div className="w-8 h-8 rounded-full border-2 border-primary border-t-transparent animate-spin" />
              <span className="text-[10px] font-black tracking-widest uppercase text-muted-foreground">Waiting for /map…</span>
            </div>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
