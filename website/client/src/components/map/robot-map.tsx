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

const MAX_ACTUAL_PATH = 2000; // max waypoints lưu lịch sử
const MIN_DIST = 0.05;        // mỗi 5cm mới ghi thêm 1 điểm

export function RobotMap() {
  const { telemetry } = useRobotState();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [navGoal, setNavGoal] = useState<{ x: number; y: number } | null>(null);
  const [mapImage, setMapImage] = useState<HTMLImageElement | null>(null);
  const [localMapInfo, setLocalMapInfo] = useState<MapInfo | null>(null);
  const actualPathRef = useRef<{ x: number; y: number }[]>([]);
  const lastRecordedRef = useRef<{ x: number; y: number } | null>(null);
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  // Ưu tiên map_info từ telemetry (live), fallback về YAML từ disk
  const mapInfo: MapInfo | null = telemetry?.map_info || localMapInfo;

  // Fetch map_info từ YAML một lần khi mount
  useEffect(() => {
    fetch(`${API_URL}/api/maps/live/info`)
      .then(r => r.json())
      .then(data => { if (data?.resolution) setLocalMapInfo(data); })
      .catch(() => {});
  }, [API_URL]);

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

  // Ghi lịch sử đường đi thực tế mỗi khi map_pose thay đổi
  useEffect(() => {
    const pose = telemetry?.map_pose;
    if (!pose) return;
    const last = lastRecordedRef.current;
    if (last) {
      const dx = pose.x - last.x;
      const dy = pose.y - last.y;
      if (Math.sqrt(dx * dx + dy * dy) < MIN_DIST) return; // chưa đi đủ xa
    }
    lastRecordedRef.current = { x: pose.x, y: pose.y };
    actualPathRef.current = [
      ...actualPathRef.current.slice(-MAX_ACTUAL_PATH + 1),
      { x: pose.x, y: pose.y }
    ];
  }, [telemetry?.map_pose]);

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

    // --- 1. Vẽ GLOBAL PLANNED PATH (xanh lá nhạt, nét đứt) ---
    const plan = telemetry?.plan;
    if (plan && plan.length > 1) {
      ctx.beginPath();
      const p0 = rosToPixel(plan[0].x, plan[0].y);
      ctx.moveTo(p0.px, p0.py);
      for (let i = 1; i < plan.length; i++) {
        const { px, py } = rosToPixel(plan[i].x, plan[i].y);
        ctx.lineTo(px, py);
      }
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
      // Đầu đích (cuối plan)
      const pEnd = rosToPixel(plan[plan.length - 1].x, plan[plan.length - 1].y);
      ctx.beginPath();
      ctx.arc(pEnd.px, pEnd.py, 4, 0, Math.PI * 2);
      ctx.fillStyle = '#22c55e';
      ctx.fill();
    }

    // --- 2. Vẽ LOCAL PLANNED PATH (vàng, nét liền mỏng) ---
    const localPlan = telemetry?.local_plan;
    if (localPlan && localPlan.length > 1) {
      ctx.beginPath();
      const lp0 = rosToPixel(localPlan[0].x, localPlan[0].y);
      ctx.moveTo(lp0.px, lp0.py);
      for (let i = 1; i < localPlan.length; i++) {
        const { px, py } = rosToPixel(localPlan[i].x, localPlan[i].y);
        ctx.lineTo(px, py);
      }
      ctx.strokeStyle = '#eab308';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // --- 3. Vẽ ACTUAL PATH đường đi thực tế (xanh dương) ---
    const actualPath = actualPathRef.current;
    if (actualPath.length > 1) {
      ctx.beginPath();
      const a0 = rosToPixel(actualPath[0].x, actualPath[0].y);
      ctx.moveTo(a0.px, a0.py);
      for (let i = 1; i < actualPath.length; i++) {
        const { px, py } = rosToPixel(actualPath[i].x, actualPath[i].y);
        ctx.lineTo(px, py);
      }
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // --- 4. Vẽ ROBOT ---
    const pose = telemetry?.map_pose || (telemetry?.odom ? { x: telemetry.odom.x, y: telemetry.odom.y, yaw: telemetry.odom.yaw || 0 } : null);
    if (pose) {
      const { px, py } = rosToPixel(pose.x, pose.y);
      // Vòng tròn robot
      ctx.beginPath();
      ctx.arc(px, py, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#ef4444';
      ctx.fill();
      ctx.strokeStyle = 'white';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      // Mũi tên hướng
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(pose.yaw) * 10, py - Math.sin(pose.yaw) * 10);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // --- 5. Vẽ NAV GOAL ---
    if (navGoal) {
      const { px, py } = rosToPixel(navGoal.x, navGoal.y);
      const cs = 5;
      ctx.beginPath();
      ctx.moveTo(px - cs, py - cs); ctx.lineTo(px + cs, py + cs);
      ctx.moveTo(px + cs, py - cs); ctx.lineTo(px - cs, py + cs);
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
        <div className="flex items-center gap-4">
          {mapInfo && (
            <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-[9px] font-mono font-bold uppercase tracking-widest text-muted-foreground/80">
              <div className="flex justify-between gap-2"><span>Size:</span> <span className="text-foreground">{mapInfo.width}×{mapInfo.height}</span></div>
              <div className="flex justify-between gap-2"><span>Res:</span> <span className="text-foreground">{mapInfo.resolution}m</span></div>
            </div>
          )}
          {/* Legend */}
          <div className="flex items-center gap-3 text-[9px] font-mono font-bold uppercase tracking-widest">
            <span className="flex items-center gap-1">
              <span className="inline-block w-4 h-0.5 bg-green-500" style={{borderTop:'2px dashed #22c55e', background:'none'}}></span>
              <span className="text-muted-foreground">Plan</span>
            </span>
            <span className="flex items-center gap-1">
              <span className="inline-block w-4 h-0.5" style={{borderTop:'2px solid #eab308'}}></span>
              <span className="text-muted-foreground">Local</span>
            </span>
            <span className="flex items-center gap-1">
              <span className="inline-block w-4 h-0.5" style={{borderTop:'2px solid #3b82f6'}}></span>
              <span className="text-muted-foreground">Actual</span>
            </span>
            <button
              onClick={() => { actualPathRef.current = []; lastRecordedRef.current = null; }}
              className="text-[8px] px-2 py-0.5 border border-border rounded hover:bg-muted/50 transition-colors"
            >Clear</button>
          </div>
        </div>
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
