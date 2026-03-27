import { MapViewer } from '@/components/map/map-viewer';

export default function SlamPage() {
  return (
    <div className="flex flex-col gap-10 h-full">
      <div className="flex items-center justify-between pb-6 border-b border-slate-200">
        <div className="flex flex-col">
           <h1 className="text-3xl font-extrabold font-sans tracking-tight text-slate-900 mb-1">
             Area Mapping
           </h1>
           <p className="text-sm font-medium text-slate-500">slam_toolbox spatial awareness node integration.</p>
        </div>
        
        <div className="hidden md:flex items-center gap-2 text-[11px] font-bold font-mono tracking-widest text-slate-600 uppercase px-4 py-2 bg-white border border-slate-200 rounded-lg shadow-sm">
          <span className="w-1.5 h-1.5 bg-slate-400 rounded-full" />
          Hardware Locked
        </div>
      </div>
      
      <div className="flex-1 min-h-[500px]">
        <MapViewer />
      </div>
    </div>
  );
}
