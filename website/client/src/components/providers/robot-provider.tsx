'use client';
import { ReactNode } from 'react';
import { RobotStateProvider } from '@/hooks/use-robot-state';
import { useROSConnection } from '@/hooks/use-ros-connection';
import { useKeyboardControl } from '@/hooks/use-keyboard-control';

// This component runs the global hooks to manage ROS connection and global keyboard control
function GlobalHooks() {
  useROSConnection();
  useKeyboardControl();
  return null;
}

export function AppProviders({ children }: { children: ReactNode }) {
  return (
    <RobotStateProvider>
      <GlobalHooks />
      {children}
    </RobotStateProvider>
  );
}
