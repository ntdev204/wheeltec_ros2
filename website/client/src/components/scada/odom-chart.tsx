'use client';
import { useEffect, useState } from 'react';
import { useRobotState } from '@/hooks/use-robot-state';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';
import { Card, CardHeader, CardTitle, CardDescription, CardAction, CardContent } from '@/components/ui/card';

export function OdomChart() {
  const { telemetry } = useRobotState();
  const [data, setData] = useState<any[]>([]);

  useEffect(() => {
    if (!telemetry) return;
    
    setData(prev => {
      const now = new Date();
      const timeStr = `${now.getMinutes()}:${now.getSeconds().toString().padStart(2, '0')}`;
      
      const newPoint = {
        name: timeStr,
        linear: Number(telemetry.odom.v_x.toFixed(2)),
        angular: Number(telemetry.odom.v_z.toFixed(2)),
      };

      const nextData = [...prev, newPoint];
      if (nextData.length > 30) nextData.shift();
      return nextData;
    });
  }, [telemetry]);

  return (
    <Card className="w-full h-[400px] flex flex-col">
      <CardHeader>
        <CardTitle>Velocity Tracking</CardTitle>
        <CardDescription>Linear X vs Angular Z</CardDescription>
        <CardAction>
          <div className="h-2 w-2 rounded-full bg-status-blue shadow-[0_0_8px_var(--status-blue)] animate-pulse" />
        </CardAction>
      </CardHeader>
      <CardContent className="flex-1 w-full relative pl-0">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={data} margin={{ top: 5, right: 10, left: 0, bottom: 5 }}>
            <CartesianGrid strokeDasharray="4 4" stroke="hsl(var(--border))" vertical={false} />
            <XAxis dataKey="name" stroke="hsl(var(--muted-foreground))" fontSize={10} tickMargin={12} tickLine={false} axisLine={false} fontFamily="monospace" />
            <YAxis stroke="hsl(var(--muted-foreground))" fontSize={10} domain={[-2, 2]} tickLine={false} axisLine={false} tickFormatter={(v) => v.toFixed(1)} fontFamily="monospace" />
            <Tooltip 
              contentStyle={{ backgroundColor: 'hsl(var(--card))', borderColor: 'hsl(var(--border))', borderRadius: '6px', fontSize: '12px' }}
              itemStyle={{ fontFamily: 'monospace', fontWeight: 'bold' }}
              labelStyle={{ color: 'hsl(var(--muted-foreground))', fontWeight: 'bold', fontSize: '10px', textTransform: 'uppercase', letterSpacing: '0.1em' }}
            />
            <Legend wrapperStyle={{ fontSize: '11px', fontWeight: 'bold', textTransform: 'uppercase', letterSpacing: '0.1em' }} iconType="circle" />
            <Line 
              type="monotone" 
              name="Linear(X)"
              dataKey="linear" 
              stroke="var(--status-orange)" 
              strokeWidth={3}
              dot={false}
              isAnimationActive={false}
            />
            <Line 
              type="monotone"
              name="Angular(Z)" 
              dataKey="angular" 
              stroke="var(--status-blue)" 
              strokeWidth={3} 
              dot={false}
              isAnimationActive={false}
            />
          </LineChart>
        </ResponsiveContainer>
      </CardContent>
    </Card>
  );
}
