'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { rosClient } from '@/lib/ros-client';
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, ArrowUpLeft, ArrowUpRight, ArrowDownLeft, ArrowDownRight, CircleDot } from 'lucide-react';
import { Card, CardHeader, CardTitle, CardDescription, CardContent, CardFooter } from '@/components/ui/card';
import { Button } from '@/components/ui/button';

export function JoystickPad() {
  const { speed, isConnected } = useRobotState();

  const sendCmd = (linear_x: number, linear_y: number) => {
    if (!isConnected) return;
    rosClient.sendCmdVel(linear_x * speed, linear_y * speed, 0);
  };

  const stop = () => rosClient.sendCmdVel(0, 0, 0);

  return (
    <Card className="w-full">
      <CardHeader>
        <CardTitle>Omni Control</CardTitle>
        <CardDescription>Holonomic drive — WASD / QEZC</CardDescription>
      </CardHeader>
      
      <CardContent className="flex-1 flex flex-col items-center">
        <div className="grid grid-cols-3 gap-3 w-fit mx-auto">
          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Move forward-left" onPointerDown={() => sendCmd(1, 1)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowUpLeft size={20} aria-hidden="true" />
          </Button>
          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Move forward" onPointerDown={() => sendCmd(1, 0)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowUp size={20} aria-hidden="true" />
          </Button>
          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Move forward-right" onPointerDown={() => sendCmd(1, -1)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowUpRight size={20} aria-hidden="true" />
          </Button>

          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Strafe left" onPointerDown={() => sendCmd(0, 1)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowLeft size={20} aria-hidden="true" />
          </Button>
          <Button size="icon" className="w-14 h-14 rounded-md shadow-md" aria-label="Stop all motors" onClick={stop}>
            <CircleDot size={20} aria-hidden="true" />
          </Button>
          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Strafe right" onPointerDown={() => sendCmd(0, -1)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowRight size={20} aria-hidden="true" />
          </Button>

          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Move backward-left" onPointerDown={() => sendCmd(-1, 1)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowDownLeft size={20} aria-hidden="true" />
          </Button>
          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Move backward" onPointerDown={() => sendCmd(-1, 0)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowDown size={20} aria-hidden="true" />
          </Button>
          <Button variant="outline" size="icon" className="w-14 h-14 rounded-md" aria-label="Move backward-right" onPointerDown={() => sendCmd(-1, -1)} onPointerUp={stop} onPointerLeave={stop}>
            <ArrowDownRight size={20} aria-hidden="true" />
          </Button>
        </div>
      </CardContent>
      <CardFooter className="flex justify-between text-[10px] text-muted-foreground uppercase tracking-widest font-semibold">
        <span>WASD: Strafe</span>
        <span>QEZC: Diag</span>
      </CardFooter>
    </Card>
  );
}
