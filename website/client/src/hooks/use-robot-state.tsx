'use client';
import { createContext, useContext, useState, ReactNode, SetStateAction } from 'react';

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
  speed: number;
  setSpeed: (speed: SetStateAction<number>) => void;
  isSetHomeMode: boolean;
  setIsSetHomeMode: (mode: SetStateAction<boolean>) => void;
}

const RobotStateContext = createContext<RobotStateContextType | undefined>(undefined);

export function RobotStateProvider({ children }: { children: ReactNode }) {
  const [isConnected, setIsConnected] = useState(false);
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null);
  const [speed, setSpeed] = useState(0.2);
  const [isSetHomeMode, setIsSetHomeMode] = useState(false);

  return (
    <RobotStateContext.Provider value={{ isConnected, setIsConnected, telemetry, setTelemetry, speed, setSpeed, isSetHomeMode, setIsSetHomeMode }}>
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
