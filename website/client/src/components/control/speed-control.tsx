'use client';
import { useRobotState } from '@/hooks/use-robot-state';
import { ROBOT_CONFIG } from '@/lib/constants';
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from '@/components/ui/card';
import { Button } from '@/components/ui/button';

export function SpeedControl() {
  const { speed, setSpeed } = useRobotState();

  const increase = () => setSpeed(Math.min(Number((speed + ROBOT_CONFIG.speedStep).toFixed(2)), ROBOT_CONFIG.maxLinearSpeed));
  const decrease = () => setSpeed(Math.max(Number((speed - ROBOT_CONFIG.speedStep).toFixed(2)), 0.05));

  const percentage = Math.round((speed / ROBOT_CONFIG.maxLinearSpeed) * 100);

  return (
    <Card className="w-full">
      <CardHeader>
        <CardTitle>Linear Speed</CardTitle>
        <CardDescription>{speed.toFixed(2)} m/s — max {ROBOT_CONFIG.maxLinearSpeed} m/s</CardDescription>
      </CardHeader>
      
      <CardContent className="flex-1 flex flex-col items-center">
        <div className="flex items-center justify-between w-full max-w-[220px] mx-auto mb-6">
          <Button 
            variant="outline"
            size="icon"
            onClick={decrease}
            aria-label="Decrease speed"
            className="w-12 h-12 rounded-md text-xl"
          >
            −
          </Button>
          
          <div className="flex flex-col items-center justify-center">
            <span className="text-3xl leading-none font-bold tracking-tight tabular-nums">{speed.toFixed(2)}</span>
            <span className="text-[10px] font-black uppercase text-primary tracking-widest mt-1">m/s</span>
          </div>
          
          <Button 
            variant="outline"
            size="icon"
            onClick={increase}
            aria-label="Increase speed"
            className="w-12 h-12 rounded-md text-xl"
          >
            +
          </Button>
        </div>
        
        <div className="w-full h-1.5 bg-muted rounded-full overflow-hidden" role="progressbar" aria-valuenow={percentage} aria-valuemin={0} aria-valuemax={100} aria-label="Speed level">
          <div 
            className="h-full bg-primary transition-[width] duration-300 ease-out" 
            style={{ width: `${percentage}%` }}
          />
        </div>
      </CardContent>
    </Card>
  );
}
