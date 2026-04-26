'use client';

import { useState, useEffect, useRef, useCallback } from 'react';
import { useRobotState } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { toast } from 'sonner';
import {
  Map, Play, Square, Save, RotateCcw,
  Wifi, WifiOff, Maximize2, ZoomIn, ZoomOut, Activity,
} from 'lucide-react';

/** Blur the currently focused element so WASD keys return to window */
function releaseButtonFocus() {
  if (document.activeElement instanceof HTMLElement) document.activeElement.blur();
}

interface MapInfo {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
}

type SlamStatus = 'idle' | 'scanning' | 'saving';

const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';
const MAP_POLL_MS = 2000;

export default function MappingPage() {
  const { telemetry, isConnected } = useRobotState();

  const [slamStatus, setSlamStatus] = useState<SlamStatus>('idle');
  const [elapsedSec, setElapsedSec] = useState(0);
  const startTimeRef = useRef<number | null>(null);

  // ── Two-canvas setup ──────────────────────────────────────────────────────
  // mapCanvasRef   → draws only the bitmap image (updated every MAP_POLL_MS)
  // robotCanvasRef → transparent overlay, draws only the robot dot (updated per telemetry)
  // Both are stacked absolutely; the robot canvas must match map canvas dimensions.
  const mapCanvasRef   = useRef<HTMLCanvasElement>(null);
  const robotCanvasRef = useRef<HTMLCanvasElement>(null);
  const mapBitmapRef   = useRef<ImageBitmap | null>(null);

  const [hasMap,   setHasMap]   = useState(false);
  const [mapInfo,  setMapInfo]  = useState<MapInfo | null>(null);
  const [zoom,     setZoom]     = useState(1);
  const [coveredCells, setCoveredCells] = useState(0);
  const [totalCells,   setTotalCells]   = useState(0);

  // ── Elapsed timer ─────────────────────────────────────────────────────────
  useEffect(() => {
    if (slamStatus !== 'scanning') return;
    startTimeRef.current = Date.now();
    const id = setInterval(() => {
      setElapsedSec(Math.floor((Date.now() - (startTimeRef.current ?? Date.now())) / 1000));
    }, 1000);
    return () => clearInterval(id);
  }, [slamStatus]);

  const fmtTime = (s: number) =>
    `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;

  // ── Map info (REST fallback) ───────────────────────────────────────────────
  useEffect(() => {
    fetch(`${API_URL}/api/maps/live/info`)
      .then(r => r.json())
      .then(d => { if (d?.resolution) setMapInfo(d); })
      .catch(() => {});
  }, []);

  const activeMapInfo: MapInfo | null = telemetry?.map_info ?? mapInfo;

  // Keep a ref so async fetchAndDraw reads latest info without stale closure
  const activeMapInfoRef = useRef<MapInfo | null>(null);
  useEffect(() => { activeMapInfoRef.current = activeMapInfo; }, [activeMapInfo]);

  // ── Sync robot-canvas dimensions to match map canvas ─────────────────────
  const syncCanvasSizes = useCallback((w: number, h: number) => {
    const mc = mapCanvasRef.current;
    const rc = robotCanvasRef.current;
    if (!mc || !rc) return;
    if (mc.width !== w || mc.height !== h) {
      mc.width  = w; mc.height  = h;
      rc.width  = w; rc.height  = h;
    }
  }, []);

  // ── Draw bitmap on the MAP canvas only ────────────────────────────────────
  const drawMapBitmap = useCallback((bitmap: ImageBitmap, w: number, h: number) => {
    const canvas = mapCanvasRef.current;
    if (!canvas) return;
    syncCanvasSizes(w, h);
    const ctx = canvas.getContext('2d');
    ctx?.drawImage(bitmap, 0, 0, w, h);
  }, [syncCanvasSizes]);

  // Smoothed yaw — prevents IMU drift from spinning the arrow when robot is stationary
  const smoothYawRef = useRef<number | null>(null);
  const ALPHA = 0.15; // lower = more smoothing, higher = more responsive

  // ── Draw robot dot on the ROBOT canvas only ───────────────────────────────
  const drawRobotOverlay = useCallback(() => {
    const canvas = robotCanvasRef.current;
    if (!canvas) return;
    const info = activeMapInfoRef.current;
    if (!info) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const pose = telemetry?.map_pose
      ?? (telemetry?.odom
        ? { x: telemetry.odom.x, y: telemetry.odom.y, yaw: telemetry.odom.yaw ?? 0 }
        : null);

    if (!pose) return;

    const px = (pose.x - info.origin.x) / info.resolution;
    const py = (info.height - 1) - (pose.y - info.origin.y) / info.resolution;

    // Exponential smoothing on yaw to absorb IMU gyro drift when stationary.
    // Handles angle wrap-around (±180° boundary).
    const rawYaw = pose.yaw;
    if (smoothYawRef.current === null) {
      smoothYawRef.current = rawYaw;
    } else {
      let delta = rawYaw - smoothYawRef.current;
      // Normalize delta to [-π, π] to handle wrap-around
      while (delta >  Math.PI) delta -= 2 * Math.PI;
      while (delta < -Math.PI) delta += 2 * Math.PI;
      smoothYawRef.current += ALPHA * delta;
    }
    const yaw = smoothYawRef.current;

    // Canvas: x→right, y→down. Map is flipud so ROS +y = canvas up.
    const ARROW_LEN = 18;
    const ax = px + Math.cos(yaw) * ARROW_LEN;
    const ay = py - Math.sin(yaw) * ARROW_LEN;


    // Glow ring
    ctx.beginPath();
    ctx.arc(px, py, 10, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(239,68,68,0.2)';
    ctx.fill();

    // Body
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#ef4444';
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Heading arrow shaft
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(ax, ay);
    ctx.strokeStyle = '#facc15';   // yellow — easy to see
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Arrowhead
    const HEAD = 6;
    const angle = Math.atan2(ay - py, ax - px);
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(ax - HEAD * Math.cos(angle - 0.4), ay - HEAD * Math.sin(angle - 0.4));
    ctx.lineTo(ax - HEAD * Math.cos(angle + 0.4), ay - HEAD * Math.sin(angle + 0.4));
    ctx.closePath();
    ctx.fillStyle = '#facc15';
    ctx.fill();

    // Debug: smoothed yaw in degrees
    const yawDeg = ((yaw * 180) / Math.PI).toFixed(1);
    const source = telemetry?.map_pose ? 'TF' : 'odom';
    ctx.font = `bold 11px monospace`;
    ctx.fillStyle = 'rgba(0,0,0,0.6)';
    ctx.fillRect(px + 12, py - 20, 92, 16);
    ctx.fillStyle = '#facc15';
    ctx.fillText(`${yawDeg}° [${source}]`, px + 14, py - 7);
  }, [telemetry]);

  // Robot overlay re-draws ONLY when telemetry changes — never touches map canvas
  useEffect(() => {
    drawRobotOverlay();
  }, [drawRobotOverlay]);

  // ── Live map polling ───────────────────────────────────────────────────────
  useEffect(() => {
    if (slamStatus !== 'scanning') return;
    let cancelled = false;

    const fetchAndDraw = async () => {
      try {
        const res = await fetch(`${API_URL}/api/maps/live/image?t=${Date.now()}`);
        if (!res.ok) {
          if (res.status === 404)
            fetch(`${API_URL}/api/maps/live/trigger`, { method: 'POST' }).catch(() => {});
          return;
        }
        const blob   = await res.blob();
        const bitmap = await createImageBitmap(blob);
        if (cancelled) { bitmap.close(); return; }

        mapBitmapRef.current?.close();
        mapBitmapRef.current = bitmap;

        const info = activeMapInfoRef.current;
        const w = info?.width  ?? bitmap.width;
        const h = info?.height ?? bitmap.height;

        drawMapBitmap(bitmap, w, h);
        // Redraw robot on top after new map frame
        drawRobotOverlay();

        if (!hasMap) setHasMap(true);
      } catch (_) {}
    };

    fetchAndDraw();
    const id = setInterval(fetchAndDraw, MAP_POLL_MS);
    return () => { cancelled = true; clearInterval(id); };
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [slamStatus]);

  // Cleanup bitmap on unmount
  useEffect(() => () => { mapBitmapRef.current?.close(); }, []);

  // ── Coverage from OccupancyGrid telemetry ────────────────────────────────
  useEffect(() => {
    const mapData = telemetry?.map?.data;
    if (!mapData) return;
    const total   = mapData.length;
    const covered = mapData.filter((v: number) => v !== -1).length;
    setCoveredCells(covered);
    setTotalCells(total);
  }, [telemetry?.map]);

  // ── SLAM handlers ─────────────────────────────────────────────────────────
  const handleStartSlam = () => {
    rosClient.send('slam_control', { action: 'start' });
    setSlamStatus('scanning');
    setElapsedSec(0);
    setHasMap(false);
    mapBitmapRef.current?.close();
    mapBitmapRef.current = null;
    // Clear both canvases
    [mapCanvasRef, robotCanvasRef].forEach(r => {
      const ctx = r.current?.getContext('2d');
      if (r.current && ctx) ctx.clearRect(0, 0, r.current.width, r.current.height);
    });
    setCoveredCells(0); setTotalCells(0);
    releaseButtonFocus();
    toast.success('SLAM Started', { description: 'Di chuyển robot để xây dựng bản đồ.' });
  };

  const handleStopSlam = () => {
    rosClient.send('slam_control', { action: 'stop' });
    setSlamStatus('idle');
    releaseButtonFocus();
    toast.info('SLAM Stopped', { description: 'Dừng quét. Lưu hoặc reset bản đồ.' });
  };

  const handleSaveMap = async () => {
    const wasScanning = slamStatus === 'scanning';
    setSlamStatus('saving');
    releaseButtonFocus();
    try {
      rosClient.send('slam_control', { action: 'save' });
      // Backend: saves map via map_saver_cli THEN stops SLAM + restarts Nav2
      // We wait for the whole sequence (save ~3s + stop ~2s + nav2 start)
      await new Promise(r => setTimeout(r, 5000));
      toast.success('Map Saved', { description: 'Bản đồ mới đã được ghi vào disk.' });
    } catch (_) {
      toast.error('Save Failed', { description: 'Không thể lưu bản đồ.' });
    } finally {
      setSlamStatus('idle');
    }
  };

  const handleResetMap = () => {
    rosClient.send('slam_control', { action: 'reset' });
    setHasMap(false);
    mapBitmapRef.current?.close();
    mapBitmapRef.current = null;
    [mapCanvasRef, robotCanvasRef].forEach(r => {
      const ctx = r.current?.getContext('2d');
      if (r.current && ctx) ctx.clearRect(0, 0, r.current.width, r.current.height);
    });
    setElapsedSec(0); setCoveredCells(0); setTotalCells(0);
    setSlamStatus('idle');
    releaseButtonFocus();
    toast.warning('Map Reset', { description: 'SLAM state đã được xóa.' });
  };

  // ── Derived stats ─────────────────────────────────────────────────────────
  const coveragePct = totalCells > 0
    ? ((coveredCells / totalCells) * 100).toFixed(1) : '0.0';

  const mapAreaM2 = activeMapInfo
    ? (coveredCells * activeMapInfo.resolution ** 2).toFixed(2) : '—';

  // ── Render ────────────────────────────────────────────────────────────────
  return (
    <div className="flex flex-col gap-6 min-h-full">

      {/* Page Header */}
      <div className="flex flex-col md:flex-row md:items-center justify-between pb-6 border-b border-border gap-4">
        <div className="flex flex-col">
          <h1 className="text-3xl font-bold tracking-tight text-foreground mb-1 flex items-center gap-3">
            <Map size={28} className="text-primary" />
            Mapping
          </h1>
          <p className="text-sm text-muted-foreground">
            Điều khiển SLAM Toolbox để quét &amp; lưu bản đồ mới.
          </p>
        </div>

        <div className="flex items-center gap-3 flex-wrap">
          <Badge
            variant={isConnected ? 'default' : 'destructive'}
            className="w-fit text-[11px] font-bold tracking-widest uppercase px-4 py-2"
          >
            {isConnected
              ? <><Wifi size={12} className="mr-1.5 inline" />Robot Online</>
              : <><WifiOff size={12} className="mr-1.5 inline" />Offline</>}
          </Badge>

          <Badge
            variant="outline"
            className={`w-fit gap-2 text-[11px] font-bold font-mono tracking-widest uppercase px-4 py-2 ${
              slamStatus === 'scanning'
                ? 'text-status-green bg-status-green-bg border-status-green/20'
                : slamStatus === 'saving'
                ? 'text-yellow-400 bg-yellow-400/10 border-yellow-400/20'
                : 'text-muted-foreground border-border bg-muted'
            }`}
          >
            <span className={`w-1.5 h-1.5 rounded-full ${
              slamStatus === 'scanning' ? 'bg-status-green animate-pulse'
              : slamStatus === 'saving' ? 'bg-yellow-400 animate-pulse'
              : 'bg-muted-foreground'
            }`} />
            {slamStatus === 'scanning'
              ? `SCANNING — ${fmtTime(elapsedSec)}`
              : slamStatus === 'saving' ? 'SAVING…' : 'SLAM IDLE'}
          </Badge>
        </div>
      </div>

      {/* Main Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 items-stretch">

        {/* Left: Live Map */}
        <div className="lg:col-span-8 flex flex-col gap-4">
          <Card className="flex flex-col h-[600px] overflow-hidden border-border shadow-sm">
            <CardHeader className="pb-3 flex flex-row items-center justify-between border-b border-border/50 shrink-0">
              <div className="flex flex-col">
                <CardTitle className="text-[11px] font-bold text-muted-foreground tracking-[0.2em] uppercase">
                  Live Map Feed
                </CardTitle>
                <span className="text-xs font-bold text-foreground mt-1">
                  SLAM Toolbox — OccupancyGrid
                </span>
              </div>

              <div className="flex items-center gap-4">
                {activeMapInfo && (
                  <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-[9px] font-mono font-bold uppercase tracking-widest text-muted-foreground/80">
                    <div className="flex justify-between gap-2">
                      <span>Size:</span>
                      <span className="text-foreground">{activeMapInfo.width}×{activeMapInfo.height}</span>
                    </div>
                    <div className="flex justify-between gap-2">
                      <span>Res:</span>
                      <span className="text-foreground">{activeMapInfo.resolution}m</span>
                    </div>
                  </div>
                )}

                {/* Zoom controls */}
                <div className="flex items-center gap-1">
                  <button
                    onClick={() => { setZoom(z => Math.min(z + 0.25, 4)); releaseButtonFocus(); }}
                    className="p-1.5 rounded hover:bg-muted/50 transition-colors text-muted-foreground hover:text-foreground"
                    title="Zoom in"
                  ><ZoomIn size={14} /></button>
                  <span className="text-[10px] font-mono w-8 text-center text-muted-foreground">
                    {zoom.toFixed(2)}×
                  </span>
                  <button
                    onClick={() => { setZoom(z => Math.max(z - 0.25, 0.5)); releaseButtonFocus(); }}
                    className="p-1.5 rounded hover:bg-muted/50 transition-colors text-muted-foreground hover:text-foreground"
                    title="Zoom out"
                  ><ZoomOut size={14} /></button>
                  <button
                    onClick={() => { setZoom(1); releaseButtonFocus(); }}
                    className="p-1.5 rounded hover:bg-muted/50 transition-colors text-muted-foreground hover:text-foreground"
                    title="Fit to view"
                  ><Maximize2 size={14} /></button>
                </div>
              </div>
            </CardHeader>

            <CardContent className="flex-1 bg-muted/10 relative p-0 overflow-hidden">
              {/* Canvas container — both canvases stacked via absolute positioning */}
              <div className="absolute inset-0 flex items-center justify-center overflow-auto">
                <div
                  style={{
                    position: 'relative',
                    transform: `scale(${zoom})`,
                    transformOrigin: 'center center',
                    display: hasMap ? 'block' : 'none',
                  }}
                >
                  {/* Bottom: map bitmap — only updated every 2s */}
                  <canvas
                    ref={mapCanvasRef}
                    style={{ imageRendering: 'pixelated', display: 'block' }}
                  />
                  {/* Top: robot overlay — updated per telemetry, transparent background */}
                  <canvas
                    ref={robotCanvasRef}
                    style={{
                      imageRendering: 'pixelated',
                      position: 'absolute',
                      top: 0, left: 0,
                      pointerEvents: 'none',
                    }}
                  />
                </div>
              </div>

              {/* Empty state overlay */}
              {!hasMap && (
                <div className="absolute inset-0 flex items-center justify-center bg-background/50 backdrop-blur-md">
                  <div className="flex flex-col items-center gap-3">
                    {slamStatus === 'scanning' ? (
                      <>
                        <div className="w-8 h-8 rounded-full border-2 border-primary border-t-transparent animate-spin" />
                        <span className="text-[10px] font-black tracking-widest uppercase text-muted-foreground">
                          Đang nhận dữ liệu map…
                        </span>
                      </>
                    ) : (
                      <>
                        <Map size={36} className="text-muted-foreground/30" />
                        <span className="text-[11px] font-black tracking-widest uppercase text-muted-foreground/50">
                          Nhấn START để bắt đầu quét bản đồ
                        </span>
                      </>
                    )}
                  </div>
                </div>
              )}

              {/* Legend */}
              {hasMap && (
                <div className="absolute bottom-3 left-3 flex items-center gap-3 bg-background/80 backdrop-blur-sm px-3 py-1.5 rounded-md border border-border text-[9px] font-mono font-bold uppercase tracking-widest pointer-events-none">
                  <span className="flex items-center gap-1">
                    <span className="w-3 h-3 rounded-sm bg-white border border-border inline-block" />
                    <span className="text-muted-foreground">Free</span>
                  </span>
                  <span className="flex items-center gap-1">
                    <span className="w-3 h-3 rounded-sm bg-neutral-500 inline-block" />
                    <span className="text-muted-foreground">Unknown</span>
                  </span>
                  <span className="flex items-center gap-1">
                    <span className="w-3 h-3 rounded-sm bg-neutral-900 inline-block" />
                    <span className="text-muted-foreground">Occupied</span>
                  </span>
                  <span className="flex items-center gap-1">
                    <span className="w-2.5 h-2.5 rounded-full bg-red-500 inline-block" />
                    <span className="text-muted-foreground">Robot</span>
                  </span>
                </div>
              )}
            </CardContent>
          </Card>

          {/* Pose bar */}
          <Card>
            <CardHeader className="pb-2 pt-4">
              <CardTitle className="text-[10px] uppercase tracking-widest text-muted-foreground font-bold border-b pb-2">
                Telemetry Pose
              </CardTitle>
            </CardHeader>
            <CardContent className="pb-4">
              <div className="flex justify-between items-center text-sm font-mono text-muted-foreground">
                <span>X: <strong className="text-foreground">
                  {(telemetry?.map_pose?.x ?? telemetry?.odom?.x ?? 0).toFixed(3)}
                </strong> m</span>
                <span>Y: <strong className="text-foreground">
                  {(telemetry?.map_pose?.y ?? telemetry?.odom?.y ?? 0).toFixed(3)}
                </strong> m</span>
                <span>Heading: <strong className="text-foreground">
                  {(((telemetry?.map_pose?.yaw ?? telemetry?.odom?.yaw ?? 0) * 180) / Math.PI).toFixed(1)}°
                </strong></span>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Right: Controls & Stats */}
        <div className="lg:col-span-4 flex flex-col gap-5">

          <Card className="border-primary/20">
            <CardHeader className="pb-4">
              <CardTitle className="flex items-center gap-2 text-primary">
                <Activity size={18} />
                SLAM Controls
              </CardTitle>
              <CardDescription>
                Khởi động, dừng và lưu quá trình xây dựng bản đồ.
              </CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-3">
              <Button
                className="w-full bg-status-green hover:bg-status-green/90 text-white font-bold tracking-wide"
                disabled={!isConnected || slamStatus !== 'idle'}
                onClick={handleStartSlam}
              >
                <Play size={14} className="mr-2" />
                Start SLAM Scanning
              </Button>

              <Button
                variant="outline"
                className="w-full border-red-500/40 text-red-400 hover:bg-red-500/10 font-bold tracking-wide"
                disabled={!isConnected || slamStatus !== 'scanning'}
                onClick={handleStopSlam}
              >
                <Square size={14} className="mr-2" />
                Stop Scanning
              </Button>

              <Button
                className="w-full font-bold tracking-wide"
                disabled={!isConnected || slamStatus === 'saving' || !hasMap}
                onClick={handleSaveMap}
              >
                {slamStatus === 'saving' ? (
                  <>
                    <div className="w-3.5 h-3.5 border-2 border-white border-t-transparent rounded-full animate-spin mr-2" />
                    Saving…
                  </>
                ) : (
                  <>
                    <Save size={14} className="mr-2" />
                    Save Map to Disk
                  </>
                )}
              </Button>

              <Button
                variant="ghost"
                className="w-full text-muted-foreground hover:text-foreground"
                disabled={slamStatus === 'saving'}
                onClick={handleResetMap}
              >
                <RotateCcw size={14} className="mr-2" />
                Reset SLAM State
              </Button>
            </CardContent>
          </Card>

          {/* Coverage stats */}
          <Card>
            <CardHeader className="pb-3">
              <CardTitle className="text-sm">Thống kê quét</CardTitle>
              <CardDescription>Dựa trên OccupancyGrid /map topic.</CardDescription>
            </CardHeader>
            <CardContent className="flex flex-col gap-4">
              <div className="flex flex-col gap-1.5">
                <div className="flex justify-between text-[11px] font-mono text-muted-foreground">
                  <span>Coverage (known cells)</span>
                  <span className="font-bold text-foreground">{coveragePct}%</span>
                </div>
                <div className="h-2 w-full bg-muted rounded-full overflow-hidden">
                  <div
                    className="h-full bg-primary rounded-full transition-all duration-500"
                    style={{ width: `${Math.min(parseFloat(coveragePct), 100)}%` }}
                  />
                </div>
              </div>

              <div className="grid grid-cols-2 gap-3">
                <StatBox label="Elapsed"    value={fmtTime(elapsedSec)} unit="mm:ss" />
                <StatBox label="Area"       value={mapAreaM2}            unit="m²"    />
                <StatBox
                  label="Map Size"
                  value={activeMapInfo ? `${activeMapInfo.width}×${activeMapInfo.height}` : '—'}
                  unit="px"
                />
                <StatBox
                  label="Resolution"
                  value={activeMapInfo ? String(activeMapInfo.resolution) : '—'}
                  unit="m/cell"
                />
              </div>
            </CardContent>
          </Card>

          {/* Instructions */}
          <Card className="border-border/50 bg-muted/20">
            <CardHeader className="pb-2">
              <CardTitle className="text-[11px] uppercase tracking-widest text-muted-foreground font-bold">
                Hướng dẫn
              </CardTitle>
            </CardHeader>
            <CardContent className="text-xs text-muted-foreground leading-relaxed space-y-2">
              <p><strong className="text-foreground">1.</strong> Nhấn{' '}
                <strong className="text-status-green">Start SLAM</strong> để khởi động SLAM Toolbox trên robot.
              </p>
              <p><strong className="text-foreground">2.</strong> Dùng trang{' '}
                <strong className="text-foreground">Omni Control</strong> hoặc joystick để di chuyển robot.
              </p>
              <p><strong className="text-foreground">3.</strong> Theo dõi bản đồ được xây dựng theo thời gian thực.</p>
              <p><strong className="text-foreground">4.</strong> Nhấn <strong className="text-foreground">Stop</strong> rồi{' '}
                <strong className="text-foreground">Save Map</strong> để ghi vào disk.
              </p>
              <p className="text-[10px] pt-1 text-muted-foreground/60">
                Map mới sẽ dùng ở trang <em>Autonomous Nav</em> sau khi restart Nav2.
              </p>
            </CardContent>
          </Card>

        </div>
      </div>
    </div>
  );
}

function StatBox({ label, value, unit }: { label: string; value: string | number; unit: string }) {
  return (
    <div className="flex flex-col gap-0.5 bg-muted/30 rounded-md px-3 py-2.5 border border-border/50">
      <span className="text-[9px] font-bold uppercase tracking-widest text-muted-foreground">{label}</span>
      <span className="text-base font-black font-mono text-foreground leading-none">{value}</span>
      <span className="text-[9px] text-muted-foreground/70 font-mono">{unit}</span>
    </div>
  );
}
