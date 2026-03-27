'use client';
import { useEffect, useRef, useState } from 'react';
import { useRobotState } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';

export function RobotMap() {
  const { telemetry } = useRobotState();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [navGoal, setNavGoal] = useState<{ x: number, y: number } | null>(null);

  // Use the LIVE map from Nav2 (OccupancyGrid from /map topic)
  // This ensures coordinates are 100% consistent with the running costmap
  const mapInfo = telemetry?.map?.info;
  const mapData = telemetry?.map?.data;

  // === COORDINATE HELPERS (using live Nav2 map info) ===
  // Forward: ROS meters -> OccupancyGrid image pixels
  //   px = (ros_x - origin_x) / resolution
  //   py = (height - 1) - (ros_y - origin_y) / resolution
  // Reverse: OccupancyGrid image pixels -> ROS meters
  //   ros_x = px * resolution + origin_x
  //   ros_y = ((height - 1) - py) * resolution + origin_y

  const rosToPixel = (rosX: number, rosY: number) => {
    if (!mapInfo) return { px: 0, py: 0 };
    const px = (rosX - mapInfo.origin.x) / mapInfo.resolution;
    const py = (mapInfo.height - 1) - (rosY - mapInfo.origin.y) / mapInfo.resolution;
    return { px, py };
  };

  const pixelToRos = (px: number, py: number) => {
    if (!mapInfo) return { rosX: 0, rosY: 0 };
    const rosX = px * mapInfo.resolution + mapInfo.origin.x;
    const rosY = ((mapInfo.height - 1) - py) * mapInfo.resolution + mapInfo.origin.y;
    return { rosX, rosY };
  };

  useEffect(() => {
    if (!canvasRef.current || !mapInfo || !mapData) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const { width, height } = mapInfo;

    // Set canvas internal resolution to map size
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }

    // Draw OccupancyGrid using ImageData (high-performance)
    const imgData = ctx.createImageData(width, height);
    for (let i = 0; i < mapData.length; i++) {
      const val = mapData[i];
      const idx = i * 4;

      if (val === -1) { // Unknown
        imgData.data[idx] = 205;
        imgData.data[idx + 1] = 205;
        imgData.data[idx + 2] = 205;
        imgData.data[idx + 3] = 255;
      } else if (val === 0) { // Free
        imgData.data[idx] = 255;
        imgData.data[idx + 1] = 255;
        imgData.data[idx + 2] = 255;
        imgData.data[idx + 3] = 255;
      } else { // Occupied (1-100)
        const gray = Math.max(0, 255 - Math.round(val * 2.55));
        imgData.data[idx] = gray;
        imgData.data[idx + 1] = gray;
        imgData.data[idx + 2] = gray;
        imgData.data[idx + 3] = 255;
      }
    }
    ctx.putImageData(imgData, 0, 0);

    // Draw Robot (from map_pose or odom fallback)
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

      // Heading arrow
      const arrowLen = 8;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(
        px + Math.cos(pose.yaw) * arrowLen,
        py - Math.sin(pose.yaw) * arrowLen
      );
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

  }, [mapInfo, mapData, telemetry, navGoal]);

  const handleMapClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!canvasRef.current || !mapInfo) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();

    // CSS pixel -> map pixel (canvas internal size = map size)
    const imgPx = (e.clientX - rect.left) * (canvas.width / rect.width);
    const imgPy = (e.clientY - rect.top) * (canvas.height / rect.height);

    // Bounds check
    if (imgPx < 0 || imgPx >= mapInfo.width || imgPy < 0 || imgPy >= mapInfo.height) return;

    // Map pixel -> ROS meters
    const { rosX, rosY } = pixelToRos(imgPx, imgPy);

    console.log(`[NAV] Click pixel=(${imgPx.toFixed(1)}, ${imgPy.toFixed(1)}) -> ROS=(${rosX.toFixed(3)}, ${rosY.toFixed(3)})`);
    console.log(`[NAV] Map info: ${mapInfo.width}x${mapInfo.height}, res=${mapInfo.resolution}, origin=(${mapInfo.origin.x}, ${mapInfo.origin.y})`);

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
            <div className="flex justify-between gap-2"><span>Origin:</span> <span className="text-foreground">{mapInfo.origin.x.toFixed(2)},{mapInfo.origin.y.toFixed(2)}</span></div>
            {navGoal && (
              <div className="flex justify-between gap-2"><span>Goal:</span> <span className="text-green-600">{navGoal.x.toFixed(2)},{navGoal.y.toFixed(2)}</span></div>
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
        {!mapData && (
          <div className="absolute inset-0 flex items-center justify-center bg-background/50 backdrop-blur-md transition-all duration-500">
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
