type MessageHandler = (data: any) => void;

class ROSClient {
  private ws: WebSocket | null = null;
  private url: string;
  private handlers: Map<string, Set<MessageHandler>> = new Map();
  private binaryHandlers: Set<MessageHandler> = new Set();
  public onConnectionChange: ((connected: boolean) => void) | null = null;
  private reconnectTimer: NodeJS.Timeout | null = null;

  constructor() {
    this.url = process.env.NEXT_PUBLIC_WS_URL || 'ws://127.0.0.1:8000/ws';
  }

  connect() {
    if (this.ws?.readyState === WebSocket.OPEN || this.ws?.readyState === WebSocket.CONNECTING) return;
    
    try {
      this.ws = new WebSocket(this.url);
      this.ws.binaryType = 'blob';

      this.ws.onopen = () => {
        console.log('Connected to Wheeltec SCADA API');
        this.onConnectionChange?.(true);
        if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
      };

      this.ws.onclose = () => {
        console.log('Disconnected from Wheeltec SCADA API');
        this.onConnectionChange?.(false);
        this.ws = null;
        // Auto-reconnect after 3s
        this.reconnectTimer = setTimeout(() => this.connect(), 3000);
      };

      this.ws.onmessage = (event) => {
        if (typeof event.data === 'string') {
          try {
            const msg = JSON.parse(event.data);
            if (msg.type && this.handlers.has(msg.type)) {
              this.handlers.get(msg.type)?.forEach(handler => handler(msg.payload));
            }
          } catch (e) {
            console.error("Failed to parse WS JSON", e);
          }
        } else {
          // Binary data (JPEG)
          this.binaryHandlers.forEach(handler => handler(event.data));
        }
      };
      
      this.ws.onerror = () => {
        // Will trigger onclose subsequently
      };
    } catch (e) {
      console.error("WebSocket connection error", e);
      this.reconnectTimer = setTimeout(() => this.connect(), 3000);
    }
  }

  on(type: string, handler: MessageHandler) {
    if (!this.handlers.has(type)) {
      this.handlers.set(type, new Set());
    }
    this.handlers.get(type)?.add(handler);
  }

  off(type: string, handler: MessageHandler) {
    this.handlers.get(type)?.delete(handler);
  }

  onBinary(handler: MessageHandler) {
    this.binaryHandlers.add(handler);
  }
  
  offBinary(handler: MessageHandler) {
    this.binaryHandlers.delete(handler);
  }

  send(type: string, payload: any) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ type, payload }));
    }
  }

  sendCmdVel(linear_x: number, linear_y: number, angular_z: number) {
    this.send('cmd_vel', { linear_x, linear_y, angular_z });
  }

  sendNavGoal(x: number, y: number, theta: number) {
    this.send('nav_goal', { x, y, theta });
  }
}

export const rosClient = new ROSClient();
