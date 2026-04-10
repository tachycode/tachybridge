export interface TopicInfo {
  name: string;
  topic_id: number;
  type: 'image' | 'pose';
}

export interface ClientOptions {
  /** Gateway WebSocket URL base, e.g., "ws://localhost:9090" */
  url: string;
  /** Room ID to join */
  roomId: string;
  /** Auto-reconnect on disconnect (default: true) */
  autoReconnect?: boolean;
  /** Reconnect interval in ms (default: 1000) */
  reconnectInterval?: number;
}

type ImageCallback = (topic: string, blob: Blob, timestamp: number) => void;
type PoseCallback = (topic: string, values: Float64Array, timestamp: number) => void;
type StatusCallback = (status: 'connected' | 'disconnected' | 'reconnecting') => void;
type QualityCallback = (topic: string, actualFps: number) => void;

const STREAM_TYPE_IMAGE = 0x01;
const STREAM_TYPE_POSE = 0x02;
const HEADER_SIZE = 6;

export class TachyBridgeClient {
  private controlWs: WebSocket | null = null;
  private imageWs: WebSocket | null = null;
  private options: Required<ClientOptions>;
  private sessionId: number = 0;
  private topics: TopicInfo[] = [];
  private topicIdMap: Map<number, TopicInfo> = new Map();
  private schemas: Map<number, { format: string; fields: string[] }> = new Map();

  private imageCallbacks: ImageCallback[] = [];
  private poseCallbacks: PoseCallback[] = [];
  private statusCallbacks: StatusCallback[] = [];
  private qualityCallbacks: QualityCallback[] = [];
  private closed = false;

  constructor(options: ClientOptions) {
    this.options = {
      autoReconnect: true,
      reconnectInterval: 1000,
      ...options,
    };
  }

  /** Connect to the gateway (control + image WebSockets) */
  async connect(): Promise<void> {
    this.closed = false;
    const { url, roomId } = this.options;

    // 1. Connect control WebSocket
    this.controlWs = new WebSocket(`${url}/room/${roomId}`);
    this.controlWs.binaryType = 'arraybuffer';

    await new Promise<void>((resolve, reject) => {
      this.controlWs!.onopen = () => resolve();
      this.controlWs!.onerror = (e) => reject(e);
    });

    // Wait for welcome message
    const welcome = await this.waitForMessage(this.controlWs, 'welcome');
    this.sessionId = welcome.session_id;
    this.topics = welcome.topics;
    for (const t of this.topics) {
      this.topicIdMap.set(t.topic_id, t);
    }

    // 2. Connect image WebSocket
    this.imageWs = new WebSocket(`${url}/room/${roomId}/image`);
    this.imageWs.binaryType = 'arraybuffer';

    await new Promise<void>((resolve, reject) => {
      this.imageWs!.onopen = () => resolve();
      this.imageWs!.onerror = (e) => reject(e);
    });

    // 3. Set up message handlers
    this.controlWs.onmessage = (e) => this.handleControlMessage(e);
    this.imageWs.onmessage = (e) => this.handleImageMessage(e);

    // 4. Disconnect handlers
    const onClose = () => {
      this.emitStatus('disconnected');
      if (!this.closed && this.options.autoReconnect) {
        this.emitStatus('reconnecting');
        setTimeout(() => this.connect(), this.options.reconnectInterval);
      }
    };
    this.controlWs.onclose = onClose;
    this.imageWs.onclose = onClose;

    this.emitStatus('connected');
  }

  /** Subscribe to a topic */
  subscribe(topicName: string): void {
    this.send({ op: 'subscribe', topic: topicName });
  }

  /** Unsubscribe from a topic */
  unsubscribe(topicName: string): void {
    this.send({ op: 'unsubscribe', topic: topicName });
  }

  /** Subscribe to all available topics */
  subscribeAll(): void {
    for (const t of this.topics) {
      this.subscribe(t.name);
    }
  }

  /** Playback control: play */
  play(): void {
    this.send({ op: 'playback_control', action: 'play' });
  }

  /** Playback control: pause */
  pause(): void {
    this.send({ op: 'playback_control', action: 'pause' });
  }

  /** Playback control: seek to timestamp (nanoseconds) */
  seek(timestampNs: number): void {
    this.send({ op: 'playback_control', action: 'seek', time_ns: timestampNs });
  }

  /** Playback control: set speed multiplier */
  setSpeed(speed: number): void {
    this.send({ op: 'playback_control', action: 'set_speed', speed });
  }

  /** Register image frame callback */
  onImage(cb: ImageCallback): void {
    this.imageCallbacks.push(cb);
  }

  /** Register pose data callback */
  onPose(cb: PoseCallback): void {
    this.poseCallbacks.push(cb);
  }

  /** Register connection status callback */
  onStatus(cb: StatusCallback): void {
    this.statusCallbacks.push(cb);
  }

  /** Register quality hint callback */
  onQuality(cb: QualityCallback): void {
    this.qualityCallbacks.push(cb);
  }

  /** Get available topics */
  getTopics(): TopicInfo[] {
    return [...this.topics];
  }

  /** Get session ID */
  getSessionId(): number {
    return this.sessionId;
  }

  /** Disconnect and clean up */
  disconnect(): void {
    this.closed = true;
    this.controlWs?.close();
    this.imageWs?.close();
    this.controlWs = null;
    this.imageWs = null;
  }

  // --- Private ---

  private send(msg: object): void {
    if (this.controlWs?.readyState === WebSocket.OPEN) {
      this.controlWs.send(JSON.stringify(msg));
    }
  }

  private handleControlMessage(event: MessageEvent): void {
    if (typeof event.data === 'string') {
      // JSON control message
      const msg = JSON.parse(event.data);
      if (msg.op === 'schema') {
        this.schemas.set(msg.topic_id, {
          format: msg.format,
          fields: msg.fields,
        });
      } else if (msg.op === 'quality_hint') {
        for (const cb of this.qualityCallbacks) {
          cb(msg.topic, msg.fps_actual);
        }
      }
    } else if (event.data instanceof ArrayBuffer) {
      // Binary pose data on control WS
      this.parseBinaryFrame(event.data);
    }
  }

  private handleImageMessage(event: MessageEvent): void {
    if (event.data instanceof ArrayBuffer) {
      this.parseBinaryFrame(event.data);
    }
  }

  private parseBinaryFrame(buffer: ArrayBuffer): void {
    if (buffer.byteLength < HEADER_SIZE) return;

    const view = new DataView(buffer);
    const streamType = view.getUint8(0);
    const topicId = view.getUint8(1);
    const tsOffsetMs = view.getUint32(2, false); // big-endian

    const topic = this.topicIdMap.get(topicId);
    if (!topic) return;

    const payload = buffer.slice(HEADER_SIZE);

    if (streamType === STREAM_TYPE_IMAGE) {
      const blob = new Blob([payload], { type: 'image/jpeg' });
      for (const cb of this.imageCallbacks) {
        cb(topic.name, blob, tsOffsetMs);
      }
    } else if (streamType === STREAM_TYPE_POSE) {
      const values = new Float64Array(payload);
      for (const cb of this.poseCallbacks) {
        cb(topic.name, values, tsOffsetMs);
      }
    }
  }

  private waitForMessage(ws: WebSocket, expectedOp: string): Promise<any> {
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => reject(new Error('Timeout waiting for ' + expectedOp)), 5000);
      const handler = (event: MessageEvent) => {
        if (typeof event.data === 'string') {
          const msg = JSON.parse(event.data);
          if (msg.op === expectedOp) {
            clearTimeout(timeout);
            ws.removeEventListener('message', handler);
            resolve(msg);
          }
        }
      };
      ws.addEventListener('message', handler);
    });
  }

  private emitStatus(status: 'connected' | 'disconnected' | 'reconnecting'): void {
    for (const cb of this.statusCallbacks) {
      cb(status);
    }
  }
}
