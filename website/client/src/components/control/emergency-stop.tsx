'use client';
import { useState, useCallback, useEffect } from 'react';
import { rosClient } from '@/lib/ros-client';
import { Button } from '@/components/ui/button';

export function EmergencyStop() {
  const [isEStopped, setIsEStopped] = useState(false);

  const handleEStop = useCallback(() => {
    rosClient.sendCmdVel(0, 0, 0);
    setIsEStopped(true);
    setTimeout(() => setIsEStopped(false), 300);
  }, []);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.code === 'Space') {
        e.preventDefault();
        handleEStop();
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleEStop]);

  return (
    <Button
      onClick={handleEStop}
      variant="outline"
      aria-label="Emergency stop — halt all motors"
      className={`relative w-full h-[96px] cursor-pointer transition-transform transition-colors duration-100 flex items-center justify-center flex-col select-none rounded-md
        ${isEStopped ? 'bg-destructive/10 border-destructive text-destructive scale-[0.98]' : 'hover:border-destructive/30 hover:bg-destructive/5 text-destructive'}`}
    >
      <span className="text-2xl font-black font-sans tracking-[0.2em] mb-1">STOP</span>
      <span className={`text-[10px] font-bold tracking-widest uppercase ${isEStopped ? 'text-destructive/80' : 'text-muted-foreground'}`}>
        Spacebar
      </span>
    </Button>
  );
}
