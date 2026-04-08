'use client';
import { createContext, useContext, useState, ReactNode, SetStateAction } from 'react';

export interface PatrolRouteWaypoint {
  x: number;
  y: number;
  yaw: number;
}

export interface PatrolRoute {
  id: number;
  name: string;
  map_id: number | null;
  waypoints: PatrolRouteWaypoint[];
  is_active: boolean;
  created_at: string;
  updated_at: string;
}

export interface PatrolSchedule {
  id: number;
  route_id: number | null;
  enabled: boolean;
  interval_minutes: number;
  loops_per_run: number;
  start_from_home: boolean;
  return_to_home: boolean;
  last_triggered_at: string | null;
  next_trigger_at: string | null;
  created_at: string;
  updated_at: string;
}

export interface PatrolRun {
  id: number;
  schedule_id: number;
  route_id: number;
  session_id: number | null;
  status: string;
  current_loop: number;
  total_loops: number;
  current_waypoint_index: number;
  started_at: string | null;
  ended_at: string | null;
  failure_reason: string | null;
  started_from_home_x: number | null;
  started_from_home_y: number | null;
  started_from_home_yaw: number | null;
  ended_at_home: boolean;
  updated_at: string;
}

export interface PatrolRuntime {
  connected: boolean;
  status: string;
  run_id: number | null;
  schedule_id: number | null;
  route_id: number | null;
  current_loop: number;
  total_loops: number;
  current_waypoint_index: number;
  total_waypoints: number;
  last_goal: PatrolRouteWaypoint | null;
  message: string | null;
  updated_at: string | null;
  battery_pct: number | null;
  charging: boolean;
  map_pose: { x: number; y: number; yaw: number } | null;
}

export interface PatrolStatusPayload {
  route: PatrolRoute | null;
  schedule: PatrolSchedule;
  latest_run: PatrolRun | null;
  runtime: PatrolRuntime;
}

export interface TelemetryData {
  odom: { x: number; y: number; z: number; v_x: number; v_y: number; v_z: number; yaw: number };
  map_pose: { x: number; y: number; yaw: number };
  imu: { ax: number; ay: number; az: number };
  voltage: number;
  charging: boolean;
  map_info?: {
    resolution: number;
    width: number;
    height: number;
    origin: { x: number; y: number };
  };
  map?: {
    info: {
      resolution: number;
      width: number;
      height: number;
      origin: { x: number; y: number };
    };
    data: number[];
  };
  plan?: { x: number; y: number }[];        // Global planned path from Nav2
  local_plan?: { x: number; y: number }[];  // Local planned path from Nav2
  battery_pct?: number;                      // Battery percentage (injected by server)
  home_position?: { x: number; y: number; yaw: number }; // Home/charger coordinates
}

interface RobotStateContextType {
  isConnected: boolean;
  setIsConnected: (connected: boolean) => void;
  telemetry: TelemetryData | null;
  setTelemetry: (data: SetStateAction<TelemetryData | null>) => void;
  patrolStatus: PatrolStatusPayload | null;
  setPatrolStatus: (status: SetStateAction<PatrolStatusPayload | null>) => void;
  speed: number;
  setSpeed: (speed: SetStateAction<number>) => void;
  isSetHomeMode: boolean;
  setIsSetHomeMode: (mode: SetStateAction<boolean>) => void;
}

const RobotStateContext = createContext<RobotStateContextType | undefined>(undefined);

export function RobotStateProvider({ children }: { children: ReactNode }) {
  const [isConnected, setIsConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);
  const [patrolStatus, setPatrolStatus] = useState<PatrolStatusPayload | null>(null);
  const [speed, setSpeed] = useState(0.2);
  const [isSetHomeMode, setIsSetHomeMode] = useState(false);

  return (
    <RobotStateContext.Provider value={{ isConnected, setIsConnected, telemetry, setTelemetry, patrolStatus, setPatrolStatus, speed, setSpeed, isSetHomeMode, setIsSetHomeMode }}>
      {children}
    </RobotStateContext.Provider>
  );
}

export function useRobotState() {
  const context = useContext(RobotStateContext);
  if (context === undefined) {
    throw new Error('useRobotState must be used within a RobotStateProvider');
  }
  return context;
}
