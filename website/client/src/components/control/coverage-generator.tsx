import { useState } from 'react';
import { toast } from 'sonner';
import { Sparkles, LoaderCircle } from 'lucide-react';

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';

export function CoverageGenerator() {
  const [isGenerating, setIsGenerating] = useState(false);
  const [robotWidth, setRobotWidth] = useState('0.5');
  const [overlap, setOverlap] = useState('0.1');
  const [pattern, setPattern] = useState('boustrophedon');

  const apiUrl = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

  const generateCoverage = async () => {
    setIsGenerating(true);
    try {
      const response = await fetch(`${apiUrl}/api/robot/coverage/generate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          robot_width: parseFloat(robotWidth),
          overlap: parseFloat(overlap),
          pattern
        }),
      });
      const body = await response.json();
      if (!response.ok) {
        throw new Error(body?.detail || 'Failed to generate coverage route');
      }
      const route = body?.route;
      toast.success('Coverage route generated', {
        description: `Created route with ${route?.waypoints?.length || 0} waypoints`
      });
    } catch (error) {
      toast.error('Coverage generation failed', {
        description: error instanceof Error ? error.message : 'Unknown error'
      });
    } finally {
      setIsGenerating(false);
    }
  };

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center gap-2">
          <div className="p-1.5 rounded-md bg-primary/10 text-primary">
            <Sparkles size={16} strokeWidth={2.5} />
          </div>
          <div>
            <CardTitle className="text-sm font-bold">Auto Coverage</CardTitle>
            <CardDescription className="text-xs">Generate full-map coverage route.</CardDescription>
          </div>
        </div>
      </CardHeader>
      <CardContent className="flex flex-col gap-3">
        <div className="grid grid-cols-2 gap-3">
          <div className="flex flex-col gap-2">
            <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Robot Width (m)</Label>
            <Input
              className="font-mono text-xs"
              value={robotWidth}
              onChange={(e) => setRobotWidth(e.target.value)}
              type="number"
              step="0.1"
              min="0.1"
              max="2.0"
            />
          </div>
          <div className="flex flex-col gap-2">
            <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Overlap</Label>
            <Input
              className="font-mono text-xs"
              value={overlap}
              onChange={(e) => setOverlap(e.target.value)}
              type="number"
              step="0.05"
              min="0.0"
              max="0.5"
            />
          </div>
        </div>

        <div className="flex flex-col gap-2">
          <Label className="text-[10px] uppercase tracking-widest text-muted-foreground">Pattern</Label>
          <Select value={pattern} onValueChange={setPattern}>
            <SelectTrigger className="text-xs">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="boustrophedon">Boustrophedon (Lawnmower)</SelectItem>
              <SelectItem value="spiral">Spiral</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <Button
          className="w-full mt-2"
          onClick={generateCoverage}
          disabled={isGenerating}
        >
          {isGenerating ? (
            <>
              <LoaderCircle size={14} className="mr-2 animate-spin" />
              Generating...
            </>
          ) : (
            <>
              <Sparkles size={14} className="mr-2" />
              Generate Coverage Route
            </>
          )}
        </Button>
      </CardContent>
    </Card>
  );
}
