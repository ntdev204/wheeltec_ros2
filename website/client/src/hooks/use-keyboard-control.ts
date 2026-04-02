'use client';
import { useEffect, useCallback } from 'react';
import { rosClient } from '@/lib/ros-client';
import { KEY_MAPPINGS, ROBOT_CONFIG } from '@/lib/constants';
import { useRobotState } from './use-robot-state';

// Map storing currently pressed keys, outside component to avoid stale closures if not careful
const activeKeys = new Set<string>();

export function useKeyboardControl() {
  const { speed, setSpeed, isConnected } = useRobotState();

  const calculateTwist = useCallback((currentSpeed: number) => {
    let linear_x = 0;
    let linear_y = 0;
    let angular_z = 0;

    // Highest priority: Emergency stop
    if (activeKeys.has(KEY_MAPPINGS.STOP)) {
      return { linear_x: 0, linear_y: 0, angular_z: 0 };
    }

    if (activeKeys.has(KEY_MAPPINGS.FORWARD)) linear_x += currentSpeed;
    if (activeKeys.has(KEY_MAPPINGS.BACKWARD)) linear_x -= currentSpeed;
    
    // Strafe (Omni handling for mecanum)
    if (activeKeys.has(KEY_MAPPINGS.LEFT)) linear_y += currentSpeed;
    if (activeKeys.has(KEY_MAPPINGS.RIGHT)) linear_y -= currentSpeed;

    // Diagonal
    if (activeKeys.has(KEY_MAPPINGS.DIAG_FL)) { linear_x += currentSpeed; linear_y += currentSpeed; }
    if (activeKeys.has(KEY_MAPPINGS.DIAG_FR)) { linear_x += currentSpeed; linear_y -= currentSpeed; }
    if (activeKeys.has(KEY_MAPPINGS.DIAG_BL)) { linear_x -= currentSpeed; linear_y += currentSpeed; }
    if (activeKeys.has(KEY_MAPPINGS.DIAG_BR)) { linear_x -= currentSpeed; linear_y -= currentSpeed; }

    // Rotation
    if (activeKeys.has(KEY_MAPPINGS.ROTATE_LEFT)) { angular_z += ROBOT_CONFIG.defaultTurn; }
    if (activeKeys.has(KEY_MAPPINGS.ROTATE_RIGHT)) { angular_z -= ROBOT_CONFIG.defaultTurn; }

    return { linear_x, linear_y, angular_z };
  }, []);

  const updateVelocity = useCallback((currentSpeed: number) => {
    const twist = calculateTwist(currentSpeed);
    rosClient.sendCmdVel(twist.linear_x, twist.linear_y, twist.angular_z);
  }, [calculateTwist]);

  useEffect(() => {
    if (!isConnected) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      
      // Ignore if typing in an input
      if (document.activeElement?.tagName === 'INPUT' || document.activeElement?.tagName === 'TEXTAREA') return;

      if (key === KEY_MAPPINGS.SPEED_UP) {
        setSpeed((s) => Math.min(Number((s + ROBOT_CONFIG.speedStep).toFixed(2)), ROBOT_CONFIG.maxLinearSpeed));
        return;
      }
      if (key === KEY_MAPPINGS.SPEED_DOWN) {
        setSpeed((s) => Math.max(Number((s - ROBOT_CONFIG.speedStep).toFixed(2)), 0.05));
        return;
      }

      if (Object.values(KEY_MAPPINGS).includes(key) && !activeKeys.has(key)) {
        activeKeys.add(key);
        updateVelocity(speed); // Using latest speed from dependency
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (activeKeys.has(key)) {
        activeKeys.delete(key);
        updateVelocity(speed);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [speed, setSpeed, updateVelocity, isConnected]);

  // Handle unmount cleanup
  useEffect(() => {
    return () => {
      activeKeys.clear();
      rosClient.sendCmdVel(0, 0, 0); 
    };
  }, []);
}
